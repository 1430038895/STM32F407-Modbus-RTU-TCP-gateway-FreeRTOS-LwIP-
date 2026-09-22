# Modbus RTU/TCP 网关（STM32F407）

基于 **STM32F407VET6 + FreeRTOS + LwIP** 的 Modbus 协议网关：上行通过以太网提供 **Modbus TCP 从站**（502 端口），下行通过 RS485 作为 **Modbus RTU 主站**轮询多台从机，在中间完成**寄存器缓存、点表采集、写穿透与单元号（站号）路由**。

## 功能特性

- **Modbus TCP 从站（上行）**：LwIP + LAN8742（RMII），监听 502 端口，响应外部 Modbus TCP 主站请求
- **Modbus RTU 主站（下行）**：RS485 总线，支持轮询 1~8 台从机（`MB_MAX_SLAVE = 8`）
- **寄存器缓存**：每台从机缓存 64 个保持寄存器（`MB_MAX_REGS = 64`），FreeRTOS 互斥锁保护并发访问
- **写穿透**：TCP 侧写请求透传下发到 RTU 总线上的目标从机
- **单元号路由**：TCP 请求中的 Unit ID 直接映射为 RTU 从机站号（1~8）
- **功能码支持**：`0x03` 读保持寄存器、`0x04` 读输入寄存器、`0x06` 写单个寄存器、`0x10` 写多个寄存器
- **数据类型**：16 位无符号 / 有符号整数、32 位浮点（支持 `ABCD` 高字在前与 `CDAB` 低字在前两种字序）
- **独立看门狗（IWDG）**：专用喂狗任务，防止主循环卡死
- **CubeMX 工程 + 双构建链**：Keil MDK-ARM 与 CMake/GCC 均可编译

## 系统架构

```
 Modbus TCP 主站（上位机 / SCADA）
              │  TCP :502
              ▼
 ┌──────────────────────────────┐
 │   Modbus TCP 从站任务 (LwIP)  │
 └──────────────┬───────────────┘
                │  寄存器缓存（互斥锁）＋ 设备点表
                │  单元号路由 / 写穿透
 ┌──────────────▼───────────────┐
 │   Modbus RTU 主站任务 (RS485) │
 └──────────────┬───────────────┘
                │  RTU : 0x03 / 0x04 / 0x06 / 0x10
        ┌───────┼───────┬────────┐
        ▼       ▼       ▼        ▼
   从机1     从机2    从机3  ...  从机8
```

## 硬件平台

| 项目 | 说明 |
| --- | --- |
| 主控 | STM32F407VET6（Cortex-M4F, 168 MHz） |
| 以太网 PHY | LAN8742A（RMII 接口） |
| 下行总线 | RS485 半双工（USART5 + MAX3485，PD1 方向控制，9600 8N1） |
| 调试口 | USART1 @ 115200 |
| 引脚分配 | 详见 [docs/hardware.md](docs/hardware.md) 与 CubeMX 工程 `modbus.ioc` |

## 硬件连接

```
电脑 ──网线── LAN8720 ──RMII── STM32F407VET6   （Modbus TCP 上行，502 端口）
电脑 ──USB── USB转RS485 ──A/B── MAX3485 ──UART5── STM32F407VET6  （Modbus RTU 下行轮询）
```

详细引脚表和接线见 [docs/hardware.md](docs/hardware.md)。

## 软件与工具链

- STM32CubeMX（HAL 库 + 代码生成）
- Keil MDK-ARM（`MDK-ARM/modbus.uvprojx`）
- 或 CMake + arm-none-eabi-gcc（`CMakeLists.txt`）
- FreeRTOS（CMSIS-RTOS V2 封装）
- LwIP（TCP/IP 协议栈）

## 目录结构

```
├── Core/                  # 应用代码（CubeMX 生成 + 自研）
│   ├── Inc/
│   │   ├── mb_gateway.h   # 网关应用层：缓存/点表/路由/写穿透
│   │   ├── mb_rtu.h       # RTU 主站驱动接口
│   │   └── mb_tcp.h       # TCP 从站任务
│   └── Src/
│       ├── mb_gateway.c
│       ├── mb_rtu.c
│       ├── mb_tcp.c
│       └── freertos.c     # 任务创建（TCP/RS485/看门狗/默认）
├── LWIP/                  # LwIP 移植层（App/Target）
├── Drivers/               # HAL 驱动 + CMSIS + LAN8742 BSP
├── Middlewares/           # FreeRTOS / LwIP 源码
├── MDK-ARM/               # Keil 工程（编译产物已忽略）
├── cmake/                 # CMake 工具链配置
├── modbus.ioc             # CubeMX 工程配置
└── CMakeLists.txt         # CMake 构建
```

## 构建方法

### Keil MDK-ARM

1. 用 Keil µVision 打开 `MDK-ARM/modbus.uvprojx`
2. 选择目标芯片 STM32F407VETx
3. 编译下载（需要先通过 CubeMX 配置好工具链路径）

### CMake + GCC（可选，需额外安装工具链）

日常开发推荐直接用上面的 Keil 工程；CMake 方式供没有 Keil 的环境使用，需要先安装
`arm-none-eabi-gcc`（GNU Arm Embedded Toolchain）和 Ninja。

```bash
cmake --preset Debug          # 配置（读取 CMakePresets.json，Ninja + GCC 工具链）
cmake --build build/Debug     # 编译，产物在 build/Debug/
```

或不用 preset 的等价写法：

```bash
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

## 使用说明

1. 按 `modbus.ioc` 完成引脚与时钟配置后重新生成代码（注意保留 `Core/Src` 下自研文件）；
2. 在 `mb_gateway.c` 的 `g_devs[]` 中维护设备点表（从机站号、寄存器地址、数据类型、字序），点表含义见 [docs/modbus-points.md](docs/modbus-points.md)；
3. 编译烧录后，用 Modbus 调试工具（如 MThings）连接：
   - 以太网侧：Modbus TCP，目标 IP 为板子地址，端口 502，Unit ID = 目标 RTU 从机站号
   - 串口侧：Modbus RTU 主站调试，可读取/写入各从机寄存器

## 版本记录

- **v1.0**：初版完整工程，实现 RTU/TCP 网关核心功能

## License

本项目基于 ST HAL / FreeRTOS / LwIP，遵循各组件原始开源许可；自研代码部分见仓库内说明。
