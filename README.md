# 水下机器人控制系统 (Underwater Robot Control System)

基于 STM32F407 + FreeRTOS 的水下机器人（ROV/AUV）嵌入式控制系统。

## 项目概述

本项目是一个水下机器人的核心控制固件，负责传感器数据采集、运动控制、多通道通信等任务。系统采用 FreeRTOS 实时操作系统进行多任务调度，保证各功能模块的实时性和稳定性。

## 硬件平台

| 项目 | 型号/参数 |
|------|----------|
| 主控芯片 | STM32F407VGT6 (Cortex-M4, 168MHz) |
| 封装 | LQFP100 |
| 开发环境 | MDK-ARM (Keil µVision 5) |
| 代码生成 | STM32CubeMX |
| 实时系统 | FreeRTOS (CMSIS-OS v2) |

## 外设资源

| 外设 | 功能 |
|------|------|
| USART1 | 蓝牙通信 / 调试输出 |
| USART2 | 预留通信接口 |
| USART3 | RF 无线通信（指令收发） |
| UART4 | 备用串口 |
| USART6 | IMU 陀螺仪数据接收 |
| TIM2 | 舵机 PWM 输出（4通道） |
| TIM3 | 电机 PWM 输出 |
| GPIO | 电机方向控制、设备初始化 |

## 传感器

| 传感器 | 型号 | 功能 |
|--------|------|------|
| IMU | 6轴陀螺仪 | 姿态角（Roll/Pitch/Yaw）、角速度 |
| MS5837 | 压力传感器 | 深度、压力、温度 |
| INA226 | 电流/功率监测 | 总线电压、电流、功率 |
| OpenMV | 视觉模块 | 图像识别与目标检测 |

## 通信模块

- **RF 无线模块**：远距离遥控指令收发
- **蓝牙模块**：近距离无线调试与参数配置
- **OpenMV**：视觉数据传输

## 运动控制

系统支持以下预设动作：

| 指令 | 说明 |
|------|------|
| STOP | 停止（舵机中位 1.5ms） |
| FORWARD | 前进 |
| BACKWARD | 后退 |
| TURN_LEFT | 左转 |
| TURN_RIGHT | 右转 |
| DIVE | 下潜 |
| SURFACE | 上浮 |

支持两种执行模式：
- **单次模式 (ACTION_MODE_ONCE)**：动作执行一次后停止
- **保持模式 (ACTION_MODE_HOLD)**：动作持续保持

## FreeRTOS 任务架构

| 任务名 | 优先级 | 栈大小 | 功能 |
|--------|--------|--------|------|
| defaultTask | 24 | 128 | 默认任务 |
| Receive_Task | 11 | 256 | 通信数据接收 |
| Sensor_Task | 12 | 512 | 传感器数据采集与处理 |
| Control_Task | 14 | 768 | 运动控制主任务 |
| Send_Task | 10 | 512 | 数据上报/发送 |
| Parse_Task | 11 | 384 | 指令解析 |
| Status_Task | 10 | 256 | 系统状态监控 |

### 消息队列

| 队列 | 大小 | 用途 |
|------|------|------|
| uart1_rx_queue | 16×64 | USART1 蓝牙接收队列 |
| uart2_rx_queue | 16×64 | USART2 接收队列 |
| uart3_rx_queue | 16×64 | USART3 RF 指令接收队列 |
| parsed_cmd_queue | 16×64 | 解析后指令队列 |
| status_queue | 16×64 | 系统状态上报队列 |

## 项目结构

```
├── Core/                       # 核心代码
│   ├── Inc/                    # 头文件
│   │   ├── FreeRTOSConfig.h    # FreeRTOS 配置文件
│   │   ├── main.h              # 主头文件
│   │   ├── gpio.h              # GPIO 配置
│   │   ├── tim.h               # 定时器配置
│   │   ├── usart.h             # 串口配置
│   │   ├── stm32f4xx_hal_conf.h # HAL 库配置
│   │   └── stm32f4xx_it.h      # 中断服务头文件
│   └── Src/                    # 源文件
│       ├── main.c              # 主程序入口
│       ├── freertos.c          # FreeRTOS 任务实现
│       ├── gpio.c              # GPIO 初始化
│       ├── tim.c               # 定时器初始化
│       ├── usart.c             # 串口初始化
│       ├── stm32f4xx_hal_msp.c # HAL MSP 配置
│       ├── stm32f4xx_it.c      # 中断服务函数
│       ├── system_stm32f4xx.c  # 系统时钟配置
│       └── stm32f4xx_hal_timebase_tim.c # HAL 时基
├── Drivers/                    # 驱动层
│   ├── CMSIS/                  # CMSIS 核心文件
│   ├── STM32F4xx_HAL_Driver/   # STM32 HAL 库
│   ├── communication/          # 通信模块驱动
│   │   ├── bluetooth.c/h       # 蓝牙驱动
│   │   ├── RF.c / RF1.h        # RF 无线驱动
│   │   └── openmv.c/h          # OpenMV 驱动
│   ├── sensors/                # 传感器驱动
│   │   ├── imu.c/h             # IMU 陀螺仪驱动
│   │   ├── msp5837.c/h         # MS5837 深度传感器驱动
│   │   ├── INA226.c/h          # INA226 功率监测驱动
│   │   ├── software_iic2.c/h   # 软件 I2C 主机
│   │   ├── software_iic3.c/h   # 软件 I2C 从机
│   │   └── sensors.c/h         # 传感器统一接口
│   └── sport/                  # 运动控制驱动
│       ├── servo.c/h           # 舵机控制
│       └── BMQ.c/h             # 电机驱动
├── Middlewares/                # 中间件
│   └── Third_Party/            # FreeRTOS 源码
├── MDK-ARM/                    # Keil 工程文件
│   ├── test.uvprojx            # 工程文件
│   ├── test.uvoptx             # 工程选项
│   └── RTE/                    # Run-Time Environment
├── global/                     # 全局定义
├── test.ioc                    # STM32CubeMX 配置文件
└── README.md
```

## 开发环境搭建

### 软件要求

- **MDK-ARM (Keil µVision 5)**：编译与调试
- **STM32CubeMX**：外设配置与代码生成
- **STM32F4 Device Family Pack**：Keil 芯片支持包

### 编译与烧录

1. 使用 Keil µVision 5 打开 `MDK-ARM/test.uvprojx`
2. 点击 **Rebuild** 编译工程
3. 通过 J-Link / ST-Link 连接目标板
4. 点击 **Download** 烧录固件

> 注意：`test.uvguix.*` 为用户特定的 Keil 配置文件，不应提交到版本控制。

## 通信协议

控制端通过 RF/蓝牙发送指令帧，格式为：

```
帧头 (GA/GL/GR/GL/GZ/S4等) + 参数 + 结束符 (\r\n)
```

系统解析后通过 `parsed_cmd_queue` 分发给 `Control_Task` 执行相应动作。

## 系统时钟

- HSE 外部晶振 → PLL → SYSCLK = 168MHz
- APB1 = 42MHz, APB2 = 84MHz
- CSS (时钟安全系统) 已启用

## 版本信息

- STM32CubeMX 版本：6.x
- HAL 驱动：STM32F4xx HAL Driver
- FreeRTOS：CMSIS-OS v2 封装

## 许可证

本项目基于 STM32CubeMX 生成，HAL 库和 FreeRTOS 部分遵循 STMicroelectronics 和 FreeRTOS 各自的许可证条款。用户自定义代码部分由开发者保留版权。
