# sim_slaves.py —— Modbus RTU 从机模拟器
# 掉线策略：从机1 永不掉线；从机2 极易掉线；从机3~8 偶尔随机掉线。
import asyncio, random, struct, time
from pymodbus.server import StartAsyncSerialServer
from pymodbus.framer import FramerType
from pymodbus.simulator import SimDevice, SimData, DataType
from pymodbus.constants import ExcCodes

PORT = "COM10"
BAUD = 115200
SIZE = 64

T0 = time.time()
RNG = random.Random()
DOWN = {}           # dev -> 恢复时刻（随机掉线用）
LASTSEC = {}        # dev -> 上次判定的秒（每秒只判一次）

def T():
    return time.time() - T0

def is_down(dev):
    """判断某从机本轮是否"掉线"（返回异常应答，网关会判为离线）"""
    if dev == 1:
        return False
    if dev == 2:
        # 从机2：每 30 秒里 10~22 秒掉线（非常容易掉）
        return 10 <= (T() % 30) <= 22
    # 从机3~8：每秒 3% 概率掉线，持续 3~8 秒
    s = int(T())
    if LASTSEC.get(dev) != s:
        LASTSEC[dev] = s
        if dev in DOWN:
            if T() >= DOWN[dev]:
                del DOWN[dev]
        elif RNG.random() < 0.03:
            DOWN[dev] = T() + RNG.uniform(3, 8)
    return dev in DOWN

def sec_val(key, lo, hi):
    """同一个 1 秒内固定，下一秒换新随机值"""
    s = int(T())
    r = random.Random((s * 2654435761 + key * 40503) & 0xFFFFFFFF)
    return r.uniform(lo, hi)

def fastcnt():
    """每毫秒都变，保证每次轮询都读到新值"""
    return int(T() * 1000) & 0xFFFF

def put_u16(r, b, v): r[b] = int(round(v)) & 0xFFFF
def put_i16(r, b, v): r[b] = int(round(v)) & 0xFFFF
def put_f32(r, b, v):
    bb = struct.pack(">f", float(v))
    r[b] = int.from_bytes(bb[0:2], "big")
    r[b+1] = int.from_bytes(bb[2:4], "big")
def put_u32dec(r, b, ip, fr):
    r[b] = (ip >> 16) & 0xFFFF
    r[b+1] = ip & 0xFFFF
    r[b+2] = int(fr) & 0xFFFF

def init_regs(dev):
    """配置类寄存器（不随时间变化，等上位机写穿透）"""
    r = [0] * SIZE
    r[0] = 9600
    r[1] = dev
    r[2] = 2
    r[3] = 250 + dev * 10
    if dev == 2:
        r[9] = 400
        r[10] = 2000
    return r

def upd_flow(dev, r, o):
    put_u16(r, o+7, fastcnt())
    put_u32dec(r, o+4, int(1000 + dev*300 + T()), int(sec_val(dev*100+4, 0, 99)))
    put_u32dec(r, o+13, int(500 + dev*50 + T()),  int(sec_val(dev*100+13, 0, 99)))
    put_u16(r, o+17, int(sec_val(dev*100+17, 0, 6000)))
    if dev == 5:
        put_u16(r, o+11, sec_val(5011, 0, 1000))
        put_u16(r, o+12, sec_val(5012, 0, 1000))
        put_u16(r, o+20, sec_val(5020, 0, 3000))
    if dev == 7:
        put_u16(r, o+8, sec_val(7008, 500, 2000))
        put_u16(r, o+19, sec_val(7019, 0, 500))

def upd_weather(r, o):
    put_f32(r, o+0, (T() * 1000) % 1000)
    put_f32(r, o+2, sec_val(302, 0, 360))
    put_f32(r, o+4, sec_val(304, -10, 40))
    put_f32(r, o+6, sec_val(306, 0, 100))
    put_f32(r, o+8, sec_val(308, 950, 1050))
    put_f32(r, o+10, sec_val(3010, 0, 5))
    put_f32(r, o+12, sec_val(3012, 0, 20))
    put_f32(r, o+14, sec_val(3014, 0, 50))
    put_f32(r, o+16, sec_val(3016, 0, 5000))

def upd_air(r, o):
    put_u16(r, o+12, fastcnt())
    put_i16(r, o+10, sec_val(4010, 150, 350))
    put_i16(r, o+11, sec_val(4011, 200, 800))
    put_u16(r, o+13, sec_val(4013, 0, 300))
    put_u16(r, o+14, sec_val(4014, 0, 500))
    put_u16(r, o+15, sec_val(4015, 300, 2000))
    put_u16(r, o+26, sec_val(4026, 0, 300))
    put_u16(r, o+27, sec_val(4027, 0, 100))

def upd_th(r, o):
    put_u16(r, o+1, fastcnt())
    put_i16(r, o+0, sec_val(8000, 200, 800))

def update_dynamic(dev, r, base):
    if dev == 3:
        upd_weather(r, base)
    elif dev in (4, 6):
        upd_air(r, base)
    elif dev == 8:
        upd_th(r, base)
    else:
        upd_flow(dev, r, base)

def make_action(dev):
    async def action(func_code, start_address, address, count, registers, values, _dev=dev):
        if is_down(_dev):
            return ExcCodes.DEVICE_FAILURE     # 掉线：返回异常，网关判为离线
        update_dynamic(_dev, registers, start_address)
        return None
    return action

DEVICES = []
for i in range(1, 9):
    DEVICES.append(SimDevice(
        id=i,
        simdata=[SimData(0, values=init_regs(i), datatype=DataType.REGISTERS)],
        action=make_action(i),
    ))

async def main():
    print("从机模拟器: 从机1永不掉线 / 从机2极易掉线 / 从机3~8偶尔掉线")
    print("串口: %s @ %d  (Ctrl+C 退出)" % (PORT, BAUD))
    await StartAsyncSerialServer(DEVICES, framer=FramerType.RTU, port=PORT,
                                 baudrate=BAUD, bytesize=8, parity="N", stopbits=1)

asyncio.run(main())
