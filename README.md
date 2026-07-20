# 两轮自平衡小车

基于 STM32F103C8T6 的自制两轮自平衡小车，采用 MPU6050 姿态传感器 + 卡尔曼滤波 + 三环串级 PID 控制算法。

## 硬件架构

```
MPU6050(IMU) ──I2C──→ STM32F103C8T6 ──PWM──→ TB6612 ──→ 直流有刷电机×2
                         │
              TIM3/TIM4(编码器) ←── 编码器AB相
                         │
              USART2 ←──→ HC-05蓝牙 ←──→ 手机APP
                         │
              I2C(软件) ──→ OLED 128×64
```

## 技术栈

| 模块 | 技术点 | 说明 |
|------|--------|------|
| 姿态解算 | 卡尔曼滤波 / DMP / 互补滤波 | `filter.c`，dt=5ms |
| 控制算法 | 三环串级 PID | `control.c`，平衡环PD + 速度环PI + 转向环PD |
| 传感器 | MPU6050 | 软件I2C驱动，INT引脚触发5ms中断 |
| 编码器 | 13线 × 4倍频 × 30减速比 | TIM3/TIM4硬件编码器模式 |
| 无线通信 | HC-05蓝牙 | USART2，115200bps |
| 显示 | OLED 128×64 | SSD1306，I2C |
| 安全保护 | 倾角>40° / 电压<10V | 自动切断电机输出 |

## PID 参数

| 控制环 | 类型 | Kp | Kd / Ki | 说明 |
|--------|------|-----|---------|------|
| 平衡环 | PD | 255.0 | 1.35 | 角度控制（核心） |
| 速度环 | PI | 160.0 | 0.80 | 编码器速度反馈（正反馈） |
| 转向环 | PD | 42.0 | 0.60 | Z轴角速度控制 |

## 运行模式

| Mode | 名称 | 说明 |
|------|------|------|
| 0 | Normal | 普通平衡模式 |
| 1 | U_Avoid | 超声波避障 |
| 2 | U_Follow | 超声波跟随 |

## 源码结构

```
├── Core/              # CubeMX生成：main.c, gpio.c, tim.c, usart.c ...
├── Hardware/          # 外设驱动
│   ├── LED/           # LED闪烁
│   ├── KEY/           # 单击/双击/长按检测
│   ├── IIC/           # 软件I2C驱动
│   ├── USART2/        # 蓝牙通信
│   ├── USART3/        # 雷达通信
│   ├── OLED/          # SSD1306显示驱动
│   ├── ENCODER/       # TIM3/TIM4编码器读取
│   └── ...
├── BALANCE/           # 平衡车核心算法
│   ├── CONTROL/       # 三环串级PID (control.c)
│   ├── MPU6050/       # 卡尔曼滤波 (filter.c)
│   ├── SHOW/          # OLED显示 + APP数据发送
│   └── DATASCOP_DP/   # 虚拟示波器数据
├── SYSTEM/            # 系统底层
└── Drivers/           # STM32 HAL库

开发环境

- MCU: STM32F103C8T6 @72MHz
- IDE: Keil MDK5
- 代码生成: STM32CubeMX
- 库版本: STM32F1 HAL Library

作者
王郁林 - 北京科技大学天津学院 通信工程 大三

- GitHub: [@heyilinq](https://github.com/heyilinq)
