# Flight Control Orientation

This document describes the current internal positive direction convention used by `FlightControl`.

## Internal Axes

After `fc_orientation` is applied, IMU samples are interpreted as:

```text
+X = forward / nose direction, roll axis
+Y = left direction, pitch axis
+Z = up direction, yaw axis
```

This matches the IMU positive-axis convention used here: when `+X` points forward, `+Y` points left, and `+Z` points up.

The rate loop reads the rotated gyro sample directly:

```cpp
gyroRoll  = sample.gyroDps.x;
gyroPitch = sample.gyroDps.y;
gyroYaw   = sample.gyroDps.z;
```

## Stick Positive Directions

Stick targets use CRSF channel centers at 1500us:

```text
Roll  positive: CH1 > 1500
Pitch positive: CH2 > 1500
Yaw   positive: CH4 > 1500
```

## Accel Convention

The angle estimator computes:

```cpp
accelRoll  = atan2(ay, az)
accelPitch = atan2(-ax, sqrt(ay * ay + az * az))
```

For a level, stationary vehicle after orientation is applied:

```text
accel.x ~= 0
accel.y ~= 0
accel.z ~= +9.8 m/s^2
```

Static tilt checks:

```text
Positive roll:  accel.y becomes positive, rollDeg becomes positive
Positive pitch: accel.x becomes negative, pitchDeg becomes positive
```

For quick orientation calibration:

```text
Level placement:   model level, accel maps to internal +Z
Nose-up placement: model stands on its tail so the expected nose / forward direction points up, accel maps to internal +X
```

Dynamic gyro checks:

```text
Positive roll rate:  gyro.x becomes positive
Positive pitch rate: gyro.y becomes positive
Positive yaw rate:   gyro.z becomes positive
```

## Calibration Order

The runtime IMU path is:

```text
raw sensor sample -> fc_orientation -> selected aircraft-frame bias/scale calibration -> gyro LPF -> estimator/PID
```

After `fc_orientation`, gyro calibration applies either the saved `gyro_bias`
or the runtime bias sampled before arming, according to `gyro_bias_mode`.
Arm-time samples update runtime control state only and are not written to
`fc.json`. Accelerometer calibration always applies
`(tf - accel_bias) * accel_scale`.
`/status.json` reports both raw sensor values (for orientation setup) and
uncalibrated `tf-*` aircraft-frame values (for IMU calibration).

## `fc_orientation` Goal

`fc_orientation` is a row-major 3x3 matrix. It should rotate the raw IMU sensor frame into the internal frame above.

The firmware applies it as:

```cpp
out.x = m[0] * in.x + m[1] * in.y + m[2] * in.z;
out.y = m[3] * in.x + m[4] * in.y + m[5] * in.z;
out.z = m[6] * in.x + m[7] * in.y + m[8] * in.z;
```

Save orientation changes to hardware config, then reboot or reinitialize flight control before validating, because the current runtime loads the matrix during initialization.

# 飞控 Orientation 方向定义

本文档说明当前 `FlightControl` 内部使用的正方向约定。

## 内部坐标轴

`fc_orientation` 应用之后，IMU 数据会按下面的内部坐标轴解释：

```text
+X = 前方 / 机头方向，roll 轴
+Y = 左侧方向，pitch 轴
+Z = 上方方向，yaw 轴
```

这和当前采用的 IMU 正方向一致：当 `+X` 指向前方时，`+Y` 指向左侧，`+Z` 指向上方。

角速度环直接读取旋转后的 gyro 数据：

```cpp
gyroRoll  = sample.gyroDps.x;
gyroPitch = sample.gyroDps.y;
gyroYaw   = sample.gyroDps.z;
```

## 摇杆正方向

摇杆目标值以 CRSF 通道 1500us 为中点：

```text
Roll  正方向：CH1 > 1500
Pitch 正方向：CH2 > 1500
Yaw   正方向：CH4 > 1500
```

## 加速度计约定

角度估计使用下面的公式：

```cpp
accelRoll  = atan2(ay, az)
accelPitch = atan2(-ax, sqrt(ay * ay + az * az))
```

因此，机体水平静止时，应用 orientation 后期望读数为：

```text
accel.x ~= 0
accel.y ~= 0
accel.z ~= +9.8 m/s^2
```

静态倾斜检查：

```text
Roll 正方向： accel.y 变为正，rollDeg 变为正
Pitch 正方向：accel.x 变为负，pitchDeg 变为正
```

快速 orientation 校准使用下面两个约束：

```text
水平放置：    模型水平，加速度映射到内部 +Z
机头朝上放置：模型尾部朝下竖起，用户预期的机头 / 前方朝上，加速度映射到内部 +X
```

动态 gyro 检查：

```text
Roll 正角速度： gyro.x 变为正
Pitch 正角速度：gyro.y 变为正
Yaw 正角速度：  gyro.z 变为正
```

## 校准顺序

运行时 IMU 数据流为：

```text
传感器原始采样 -> fc_orientation -> 所选的机体系零偏/比例校准 -> 陀螺仪低通 -> 姿态估计/PID
```

`fc_orientation` 之后，陀螺仪会根据 `gyro_bias_mode` 使用已保存的 `gyro_bias`
或解锁前采集的运行时零偏。解锁采样只更新本次运行的控制状态，不写入 `fc.json`。
加速度计校准始终使用 `(tf - accel_bias) * accel_scale`。`/status.json` 同时提供用于安装方向设置的原始传感器数据，
以及用于 IMU 校准的未校准 `tf-*` 机体系数据。

## `fc_orientation` 的目标

`fc_orientation` 是一个行主序的 3x3 矩阵。它的作用是把 IMU 原始传感器坐标系旋转到上面定义的飞控内部坐标系。

固件中的应用方式是：

```cpp
out.x = m[0] * in.x + m[1] * in.y + m[2] * in.z;
out.y = m[3] * in.x + m[4] * in.y + m[5] * in.z;
out.z = m[6] * in.x + m[7] * in.y + m[8] * in.z;
```

修改 orientation 后需要保存到 hardware config，然后重启设备或重新初始化 flight control 后再验证。当前运行时是在初始化阶段加载矩阵的。
