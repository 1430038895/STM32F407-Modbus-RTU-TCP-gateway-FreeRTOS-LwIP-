# write_one.py —— 通过 Modbus TCP(502) 写一个寄存器，然后从 5000 读回验证
#
# 寻址规则：单元号(unit) = 从机号(1..8)，寄存器地址 = 从机里的寄存器地址
# 例如设备1(COM10-001) 的 K 在地址 3：UNIT=1, ADDR=3
#
# 注意：写是“写穿透”，网关会转发到 RS485 从机；**必须真的有从机应答**，
#       否则返回异常（0x86）。没有从机时写不成功，属正常。
#
# 用法：python write_one.py

import socket
import struct
import time

HOST = "192.168.77.100"
MB_PORT = 502       # Modbus TCP
DUMP_PORT = 5000    # 文本导出（读回验证用）

UNIT = 1            # 从机号
ADDR = 3            # 寄存器地址（例：设备1 的 K）
VALUE = 250         # 写入值（K=250 -> 显示 2.50）


def mb_write_single(unit, addr, value):
    tid = 1
    pdu = struct.pack(">BHH", 0x06, addr, value)
    mbap = struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit)
    s = socket.create_connection((HOST, MB_PORT), timeout=5)
    s.sendall(mbap + pdu)
    resp = s.recv(256)
    s.close()
    if len(resp) >= 9 and (resp[7] & 0x80):
        print("写失败：异常码 0x%02X（从机没应答？地址非法？）" % resp[8])
        return False
    print("写成功，应答:", resp.hex(" "))
    return True


def read_dump():
    s = socket.create_connection((HOST, DUMP_PORT), timeout=10)
    data = b""
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        data += chunk
    s.close()
    return data.decode("utf-8", "replace")


def main():
    print("写入 unit=%d addr=%d value=%d ..." % (UNIT, ADDR, VALUE))
    mb_write_single(UNIT, ADDR, VALUE)

    time.sleep(1.5)   # 等一下，让轮询刷一轮
    text = read_dump()
    for ln in text.splitlines():
        if ln.strip():
            print(ln)


if __name__ == "__main__":
    main()
