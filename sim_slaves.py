# sim_slaves.py —— 用 pymodbus 模拟 1~8 号 Modbus RTU 从机（一个 COM 口搞定）
#
# 安装：  pip install pymodbus pyserial
# 运行：  1) 先把 MThings 关掉，保证这个 COM 口没被占用
#         2) 改下面 PORT 为你 USB-RS485 的那个 COM 口
#         3) python sim_slaves.py
#
# 说明：网关轮询用功能码 0x03（保持寄存器）。这里把每台从机的保持寄存器
#       按你给的表格填好，值都按各自倍率设成好认的数（见注释）。

import struct
import sys

# ---------------- pymodbus 导入（兼容 2.x / 3.x） ----------------
try:
    from pymodbus.server import StartSerialServer            # 3.x
except ImportError:
    from pymodbus.server.sync import StartSerialServer       # 2.x

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusSlaveContext,
    ModbusServerContext,
)

try:
    from pymodbus.framer import FramerType
    FRAMER = FramerType.RTU
except Exception:
    try:
        from pymodbus.transaction import ModbusRtuFramer as FRAMER
    except Exception:
        FRAMER = "rtu"

# ---------------- 配置 ----------------
PORT = "COM10"      # ← 改这里！你 USB-RS485 的串口号
BAUD = 9600
SIZE = 64           # 每台从机保持寄存器块大小（覆盖到地址 27 就够）


def f32(v):
    """把一个 float 转成大端两个寄存器 [高字, 低字]（ABCD）"""
    b = struct.pack(">f", v)
    return [int.from_bytes(b[0:2], "big"), int.from_bytes(b[2:4], "big")]


def make_slave(overrides):
    regs = [0] * SIZE
    for addr, v in overrides.items():
        if isinstance(v, list):
            for k, w in enumerate(v):
                regs[addr + k] = w
        else:
            regs[addr] = v
    # zero_mode=True：请求地址 N 直接对应块下标 N（避免 pymodbus 默认 +1 偏移）
    return ModbusSlaveContext(hr=ModbusSequentialDataBlock(0, regs), zero_mode=True)


# ---------------- 8 台从机的寄存器值 ----------------
# 键=寄存器地址，值=数值；列表=连续多个寄存器（用于累积量的 高/低/小数位）
SLAVES = {
    # 从机1 COM10-001 流量计
    1: make_slave({
        0: 9600, 1: 1, 2: 2, 3: 250,          # baud, station, decimals, K(=2.50)
        4: [0, 1234, 25], 7: 77,              # flow_total=1234.25, pulse
        13: [0, 567, 8], 16: 88,              # flow_temp=567.08
        17: 1234, 18: 99,                     # flow_instant=12.34
    }),
    # 从机2 COM10-002
    2: make_slave({
        0: 9600, 1: 2, 2: 2, 3: 260,          # K=2.60
        4: [0, 2345, 50], 7: 70,              # flow_total=2345.50
        9: 400, 10: 2000,                     # ma_4ma=4.00, ma_20ma=20.00
        13: [0, 678, 15], 16: 80,             # flow_temp=678.15
        17: 1567, 18: 90,                     # flow_instant=15.67
    }),
    # 从机3 COM10-003 气象站（全部 F32）
    3: make_slave({
        0:  f32(3.14), 2:  f32(180.0), 4:  f32(23.5),
        6:  f32(45.6), 8:  f32(1013.25), 10: f32(0.5),
        12: f32(3.2),  14: f32(12.0),   16: f32(1234.5),
    }),
    # 从机4 COM10-004 空气环境
    4: make_slave({
        1: 4, 2: 9600,
        10: 235, 11: 456,                     # temp=23.5, humidity=45.6
        12: 10, 13: 25, 14: 40, 15: 800,      # pm1,pm25,pm10,co2
        26: 50, 27: 33,                       # hcho=0.50, voc=3.3
    }),
    # 从机5 COM10-005 流量+压力
    5: make_slave({
        0: 9600, 1: 5, 2: 2, 3: 270,          # K=2.70
        4: [0, 3456, 20], 7: 75,              # flow_total=3456.20
        11: 100, 12: 200,                     # press_A, press_B
        13: [0, 890, 10], 16: 85,             # flow_temp=890.10
        17: 1789, 18: 95,                     # flow_instant=17.89
        20: 1013,                             # pressure_kpa=10.13
    }),
    # 从机6 COM10-006 空气环境（同 004）
    6: make_slave({
        1: 6, 2: 9600,
        10: 240, 11: 460,                     # temp=24.0, humidity=46.0
        12: 11, 13: 26, 14: 41, 15: 810,
        26: 55, 27: 35,                       # hcho=0.55, voc=3.5
    }),
    # 从机7 COM10-007 流量+NTC
    7: make_slave({
        0: 9600, 1: 7, 2: 2, 3: 280,          # K=2.80
        4: [0, 4567, 30], 7: 70, 8: 1000,     # flow_total=4567.30, ntc_res=1000
        13: [0, 901, 5], 16: 80,              # flow_temp=901.05
        17: 1801, 18: 90,                     # flow_instant=18.01
        19: 268,                              # temp_c=26.8
    }),
    # 从机8 COM10-008 简易温湿度
    8: make_slave({
        0: 327,    # humidity = 32.7 (int16 /10)
        1: 301,    # temp     = 30.1 (uint16 /10)
    }),
}

# single=False + 字典：一个串口服务器按单元号分发给 1~8 号
CONTEXT = ModbusServerContext(slaves=SLAVES, single=False)


def main():
    print("Modbus RTU 从机模拟器 (1~8 号) 启动")
    print("  串口 : %s  波特率: %d  8/N/1" % (PORT, BAUD))
    print("  按 Ctrl+C 退出")
    StartSerialServer(
        CONTEXT,
        port=PORT,
        framer=FRAMER,
        baudrate=BAUD,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=1,
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已退出")
        sys.exit(0)
