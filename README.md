# balance-car-hal

两轮自平衡小车（寒假平衡车第二版），基于 **STM32F103C8T6 + STM32Cube HAL 库**。

## 功能

- MPU6050（DMP）姿态解算，串级 PID 直立控制
- TIM1 双路 PWM 驱动电机（TB6612）
- TIM2 / TIM4 编码器测速，构成速度环
- OLED 实时显示编码器读数与状态
- 超声波测距（sr04）与循迹（xunji）模块
- UART3 串口通信

## 硬件

| 项 | 说明 |
|---|---|
| 主控 | STM32F103C8T6（64 KB Flash / 20 KB RAM） |
| 姿态 | MPU6050，I2C1，使用 DMP 硬件解算 |
| 电机 | TIM1_CH1 / TIM1_CH4 双路 PWM |
| 测速 | TIM2、TIM4 编码器模式 |
| 显示 | OLED（I2C） |
| 串口 | UART3 |

## 目录

```
Core/
  Src/         main.c、pid.c、motor.c、encoder.c、oled.c、sr04.c、xunji.c
  Inc/         对应头文件
MyCode/        MPU6050 驱动与 DMP 固件（inv_mpu 系列）
Drivers/       STM32F1 HAL 驱动 + CMSIS（已裁剪，仅保留编译所需部分）
MDK-ARM/       Keil 工程（OLED.uvprojx）
OLED.ioc       STM32CubeMX 工程文件
```

## 编译

1. Keil MDK 打开 `MDK-ARM/OLED.uvprojx`
2. 确认已安装 `STM32F1xx_DFP` 器件包
3. Build → 下载到板子

也可以双击 `OLED.ioc` 用 STM32CubeMX 重新生成代码。

## 说明

- 仓库不含编译产物（`Objects/`、`*.o`、`*.axf` 等），clone 后需自行编译
- `Drivers/CMSIS` 只保留了 `Include`、`Device/ST/STM32F1xx`、`Core`，
  剔除了用不到的 `DSP`、`Lib`、`NN`、`RTOS`（约 45 MB）
- 原始工程里的「2025-2026 寒假考核」资料（71 MB）未纳入仓库
- 第一版（标准库实现）不在本仓库中
