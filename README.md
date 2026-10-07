# STM32F407 Modbus RTU ⇄ TCP 协议转换网关

> 基于 **STM32F407VET6 + FreeRTOS + LwIP** 的 **RS485(Modbus RTU 主站) ⇄ 以太网(Modbus TCP 从站)**
> 协议转换网关，带 **TFT 本地界面**（摇杆操作）与 **网页上位机**（实时读 + 写）。

![MCU](https://img.shields.io/badge/MCU-STM32F407VET6-03234B)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-blue)
![Stack](https://img.shields.io/badge/Stack-LwIP-green)
![IDE](https://img.shields.io/badge/IDE-Keil%20MDK-orange)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

---

## 亮点（作品集）

- **协议栈**：手写 Modbus **RTU 主站**（0x03/0x04/0x06/0x10）+ **TCP 从站**（MBAP），含 CRC16、异常码、超时/重试。
- **并发架构**：FreeRTOS 多任务 + 互斥锁 + 事件通知；RS485 总线与缓存分别加锁，读写与显示互不阻塞。
- **健壮性**：UART 中断 + 环形缓冲收帧、按长度定长收帧、客户端收发超时、**掉线分级退避**、看门狗、接收自愈。
- **性能**：轮询"合并段"减少总线事务（19→8 笔）、整轮耗时约 **130ms**（8 台设备）。
- **人机界面**：ST7789 SPI TFT（**DMA 刷屏**）+ 自绘中文界面（字模自动生成，无 LVGL 依赖）+ 摇杆交互。
- **上位机联动**：PC 端 Python 桥（WebSocket）+ 网页，可**实时查看并改值**，写操作**穿透**到真实从机。

---

## 1. 它做什么

```
 8 台 RS485 从机 ──(Modbus RTU)──►  STM32 网关  ──(Modbus TCP 从站, 502)──►  SCADA / 网页
                                     │
                                     ├── TFT 本地界面（摇杆切换从机）
                                     └── 文本导出(TCP 5000) ──► PC 桥 ──► 网页
```

- **下行**：网关作为 RTU **主站**，轮询 8 台从机（流量计 / 气象站 / 空气 / 温湿度…）。
- **上行**：网关作为 Modbus TCP **从站**（端口 502）对外，**单元号 = 从机号，寄存器地址 = 从机寄存器地址**；
  读从缓存返回，写则**穿透**到 RS485 从机。
- **本地界面**：左侧 8 台从机列表，右侧显示选中从机数据（中文标签）；摇杆上下切换；右上角显示整轮轮询耗时。
- **网页上位机**：深色工业风，左列表 + 右数据；数值框改动即同步写入。

---

## 2. 系统架构

```
                 ┌──────────────────────── STM32F407 网关 ────────────────────────┐
                 │                                                                 │
  8 台从机        │  mb_rtu                                                          │
  (RS485)        │ (RTU 主站 + CRC)     ┌────────► pt[]  按“点”：值/质量/时间         │
     ────────────┼──────┐              │              │                              │
                 │      ▼              │              ├──► ui.c     本地 TFT 界面     │
                 │  mbgw_poll_task ──解码              └──► cache_dump  导出(5000)    │
                 │      │              │                                             │
                 │      └──镜像────────┴──► g_slave_regs[][] 按“从机+寄存器”          │
                 │                                  │                               │
                 │                                  ▼                               │
                 │                            mb_tcp (502)  Modbus TCP 从站           │
                 └──────────────────────────────────────────────────────────────────┘
                     ▲ 读缓存 / 写穿透                                   ▲
                     │                                                   │
             pc/bridge.py ── 读 5000 / 写 502 ──► 浏览器网页(WebSocket)
             pc/sim_slaves.py  pymodbus 从机模拟器 (USB-RS485)
```

---

## 3. 硬件与引脚

| 模块 | 信号 | 引脚 | 说明 |
|---|---|---|---|
| 调试串口 | USART1 TX/RX | PA9 / PA10 | 115200，日志 |
| RS485 | UART5 TX/RX | PC12 / PD2 | 115200 8N1，接 MAX3485 |
| RS485 | 收发方向 DE/RE | PD1 | 高=发，低=收 |
| 以太网 | RMII(LAN8720) | PA1 PA2 PA7 PC1 PC4 PC5 PB11 PB12 PB13 | 静态 IP |
| TFT ST7789 | SCK / MOSI | PB3 / PB5 | SPI1（重映射），只写 |
| TFT ST7789 | CS / DC / RST / BLK | PB0 / PB1 / PB2 / PB10 | 控制脚 |
| 摇杆 | VRx / VRy / SW | PA0 / PA3 / PB8 | ADC1_IN0 / IN3，SW 上拉 |

- 屏幕像素数据走 **SPI1_TX DMA（DMA2 Stream3 / Ch3）**，代码里配置，不依赖 CubeMX。
- 默认 IP：`192.168.77.100/24`；PC 需同网段（如 `192.168.77.10`）。

---

## 4. 目录结构

```
modbus/
├─ Core/
│  ├─ Inc/  mb_gateway.h  mb_rtu.h  mb_tcp.h  st7789.h  joystick.h  ui.h
│  │        font8x16.h  zh16.h  zh_labels.h   （字模，脚本自动生成）
│  └─ Src/  mb_gateway.c  mb_rtu.c  mb_tcp.c  st7789.c  joystick.c  ui.c
│           freertos.c  main.c  + CubeMX 底层(usart/spi/adc/gpio/iwdg...)
├─ Drivers/  Middlewares/  LWIP/     （STM32 HAL / FreeRTOS / LwIP）
├─ MDK-ARM/  modbus.uvprojx          （Keil 工程）
├─ docs/     hardware.md  modbus-points.md
├─ pc/
│  ├─ sim_slaves.py                  （Modbus RTU 从机模拟器，pymodbus）
│  └─ bridge.py                      （PC 桥 + 网页，直接运行）
├─ README.md
└─ modbus.ioc                        （CubeMX 配置）
```

### 核心文件职责

| 文件 | 职责 |
|---|---|
| `Core/Src/mb_rtu.c` | RS485 物理层 + RTU 主站：收发/CRC/中断环形缓冲 |
| `Core/Src/mb_gateway.c` | **核心**：描述表 + 点表 + 寄存器镜像 + 轮询 + 取数 + 导出 + 变更通知 |
| `Core/Src/mb_tcp.c` | Modbus TCP 从站（502） |
| `Core/Src/st7789.c` / `joystick.c` / `ui.c` | 屏幕驱动 / 摇杆 / 自绘界面 |
| `Core/Src/freertos.c` | 任务创建、模块初始化、看门狗、栈溢出钩子 |

---

## 5. 数据模型：三张表

| 表 | 位置 | 组织方式 | 用途 |
|---|---|---|---|
| **描述表** `SEG_Dx / PT_Dx / SLAVES` | Flash | 每台从机有哪些"段"、每段每个寄存器是什么点 | 初始化解析一次 |
| **点表** `pt[]` | RAM | 按"点"：值 / 质量 / 时间戳 | 屏幕、导出、`dp_*` |
| **寄存器镜像** `g_slave_regs[][]` | RAM | 按"从机 + 寄存器地址" | Modbus TCP 透传 |

点类型：`T_U16` / `T_I16` / `T_U32` / `T_U32_DEC`(累积量,3 寄存器) / `T_F32`(存原始字节，显示时整数换算，不用浮点)。
8 台设备共 **94 行描述 → 78 个输出点**；轮询时每台设备**合并成一段一次读完**。

---

## 6. 任务与并发

| 任务 | 优先级 | 作用 |
|---|---|---|
| `rs485Task` | AboveNormal | 轮询从机（总线） |
| `wdTask` | AboveNormal | 看门狗 |
| `defaultTask` | Normal | LwIP 初始化 + 屏幕界面 |
| `dumpTask` | Normal | 导出服务(5000) |
| `tcpTask` | BelowNormal | Modbus TCP(502) |

- 锁：`s_busMutex`（RS485 总线）、`s_regMutex`（缓存/点表）；
- 通知：`osEventFlags` + 脏位图，供屏幕按需刷新；
- 健壮性：**掉线分级退避**（连续失败 ≥3 每 2 轮、≥6 每 5 轮、≥12 每 12 轮轮询一次），避免坏从机拖慢整轮。

---

## 7. 编译与烧录（Keil）

1. Keil MDK 打开 `MDK-ARM/modbus.uvprojx`；
2. **Rebuild All** → 下载；
3. 串口(115200)应打印：
   `[DUMP] listening on 5000` / `[MB] Modbus TCP listening on 502`。

> 若用 CubeMX 重新生成，请注意会被覆盖的项：UART5 波特率 **115200**、FreeRTOS `defaultTask` 栈 **2048 字节**、PB8 **上拉**。

---

## 8. PC 端工具（`pc/`）

### 8.1 `pc/sim_slaves.py` —— Modbus RTU 从机模拟器
用 `pymodbus` 模拟 8 台从机（寄存器值与网关描述表一致），用于无真实设备时联调。

- 依赖：`pip install pymodbus`
- 改 `PORT` 为你的 USB-RS485 串口（默认 `COM10`，115200）；
- **掉线策略**（便于测试健壮性）：从机 1 永不掉线；从机 2 极易掉线；从机 3~8 偶尔随机掉线。

### 8.2 `pc/bridge.py` —— PC 桥 + 网页上位机（一个文件）
把网关上"文本导出(5000)"的数据通过 **WebSocket** 推给网页，并把网页上的改动**写回网关(502)**。
网页内容**内嵌在脚本里**，无需额外文件。

- 依赖：`pip install websockets`
- 运行后浏览器打开 **`http://127.0.0.1:8001`**；
- 左侧点选从机，右侧数值框**改动回车**即写入（写穿透到从机，屏幕与网页同步刷新）。

> 说明：每次给板子重新烧录后，USB-RS485 会复位，请**重启一次 `sim_slaves.py`**。

---

## 9. 说明与取舍

- **不用浮点**：`T_F32` 只存 4 个原始字节，显示端用整数位运算换算，MCU 上零浮点依赖。
- **合并段读**：会读到段间未使用的寄存器（我们清楚表结构，故安全）；若真实从机对未实现地址返回异常，需改回分段读。
- **写穿透**：写会短暂占用 RS485 总线；读对总线零占用（走缓存）。
- **lwIP 长连接**：PC 桥采用长连接并对收发加超时，避免长时间运行连接数耗尽。

---

## 10. 许可证

MIT License（见 `LICENSE`）。
