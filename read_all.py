# read_all.py  —— 从 STM32 网关（modbus）读出全部缓存点
#
# 用法：
#   1) 电脑网口 IP = 192.168.77.10，掩码 255.255.255.0（和板子 192.168.77.100 同网段，别走 WiFi）
#   2) 运行：  python read_all.py
#
# 协议（端口 5000）：连上后先收 1 字节“设备数 N”；
#   然后对每个设备 d(0..N-1)：发 1 字节 d -> 收 4 字节长度 + 该设备正文。
#   每行：点号,设备名,从机号,点名,值,q=质量,age=龄期ms
#   q=0 还没读到   q=1 好   q=2 通信失败
#   F32:xxxxxxxx 是浮点的 4 个原始字节（没做浮点解释）

import socket
import sys
import time

HOST = "192.168.77.100"
PORT = 5000
TIMEOUT = 10.0          # 单次读取超时（秒）
RETRIES = 3             # 失败重试次数


def recv_exact(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise EOFError("连接被关闭，只收到 %d/%d 字节" % (len(buf), n))
        buf += chunk
    return buf


def fetch_all():
    s = socket.create_connection((HOST, PORT), timeout=TIMEOUT)
    s.settimeout(TIMEOUT)
    try:
        ndev = recv_exact(s, 1)[0]
        lines = []
        for d in range(ndev):
            s.sendall(bytes([d]))          # 要第 d 个设备
            hdr = recv_exact(s, 4)
            n = int.from_bytes(hdr, "big")
            payload = recv_exact(s, n) if n > 0 else b""
            for ln in payload.decode("utf-8", "replace").splitlines():
                if ln.strip():
                    lines.append(ln)
        return lines
    finally:
        s.close()


def main():
    lines = None
    for i in range(1, RETRIES + 1):
        try:
            lines = fetch_all()
            if lines:
                break
            print("第 %d 次：读到 0 个点，重试..." % i)
        except OSError as e:
            print("第 %d 次失败：%s，重试..." % (i, e))
        time.sleep(1)

    if not lines:
        print("没读到数据。检查：")
        print("  - 电脑 IP 是不是 192.168.77.10？能不能 ping 通 192.168.77.100？")
        print("  - 板子串口有没有打印 [DUMP] listening on 5000 / client connected ?")
        sys.exit(1)

    for ln in lines:
        print(ln)

    print("-" * 44)
    print("共读到 %d 个点" % len(lines))
    bad = sum(1 for ln in lines if "q=1" not in ln)
    if bad:
        print("其中 %d 个点还没读到/通信失败（q=0 或 q=2）" % bad)


if __name__ == "__main__":
    main()
