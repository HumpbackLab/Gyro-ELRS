# Gyro-ExpressLRS (GLRS)

[English](#english) | [中文](#中文)

## English

Gyro-ExpressLRS, or **GLRS**, is an open-source radio link and basic flight-control project based on [ExpressLRS 3.x](https://github.com/ExpressLRS/ExpressLRS).

While retaining the low-latency radio link capabilities of ExpressLRS, GLRS adds gyroscope support and basic flight-control functions to the receiver. Its goal is to integrate radio reception, attitude sensing, and basic stabilization control into the same hardware and firmware.

### Features

- Radio control and telemetry link based on ExpressLRS 3.x
- Receiver-side gyroscope and accelerometer data acquisition
- Configurable IMU orientation and attitude estimation
- Basic rate control, attitude stabilization, and PID control
- Motor and servo mixing outputs
- Web-based configuration, runtime status, and debugging tools
- GLRS Configurator for desktop and mobile platforms (work in progress)

### Project Status

GLRS is currently under active development and validation. Its flight-control functions are primarily intended for experimentation, learning, and specific hardware integrations. IMU support, pin assignments, and output capabilities vary between receivers. Always perform thorough bench tests before flight, and verify sensor orientation, failsafe behavior, and output ranges.

### Relationship to ExpressLRS

GLRS is based on ExpressLRS 3.x and extends the receiver with gyroscope and basic flight-control capabilities. This README does not repeat the complete ExpressLRS feature set, hardware compatibility information, or usage instructions. For those details, please refer to the upstream project:

- [ExpressLRS Website and Documentation](https://www.expresslrs.org/)
- [ExpressLRS GitHub Repository](https://github.com/ExpressLRS/ExpressLRS)
- [ExpressLRS Configurator](https://github.com/ExpressLRS/ExpressLRS-Configurator)

### License

This project retains the open-source license of the upstream project. See [LICENSE](LICENSE) for details.

---

## 中文

Gyro-ExpressLRS，简称 **GLRS**，是一个基于 [ExpressLRS 3.x](https://github.com/ExpressLRS/ExpressLRS) 开发的开源遥控链路与基础飞控项目。

本项目在保留 ExpressLRS 低延迟遥控链路能力的基础上，为接收机增加了陀螺仪支持和基础飞控功能，目标是让遥控接收、姿态感知与基础稳定控制能够运行在同一套硬件和固件中。

### 主要功能

- 基于 ExpressLRS 3.x 的遥控与遥测链路
- 接收机端陀螺仪与加速度计数据采集
- IMU 安装方向配置与姿态估计
- 基础角速度环、姿态稳定和 PID 控制
- 电机/舵机混控输出
- Web 配置、运行状态查看与调试支持
- 配套的 GLRS Configurator 桌面/移动端配置工具（开发中）

### 项目状态

GLRS 目前仍在开发和验证阶段，飞控功能主要面向实验、学习和特定硬件适配。不同接收机的 IMU、引脚和输出能力可能不同，请在实际飞行前充分进行台架测试，并确认传感器方向、失控保护和输出范围设置正确。

### 与 ExpressLRS 的关系

GLRS 以 ExpressLRS 3.x 为基础，并在接收机侧扩展陀螺仪和基础飞控能力。本 README 不再重复介绍 ExpressLRS 的完整功能、硬件兼容性和使用方法；相关内容请参考上游项目：

- [ExpressLRS 官方网站与文档](https://www.expresslrs.org/)
- [ExpressLRS GitHub 仓库](https://github.com/ExpressLRS/ExpressLRS)
- [ExpressLRS Configurator](https://github.com/ExpressLRS/ExpressLRS-Configurator)

### 开源许可

本项目沿用上游项目的开源许可，详情请参阅 [LICENSE](LICENSE)。
