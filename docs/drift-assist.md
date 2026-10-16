# Drift assistance / 漂移辅助

Experimental throttle reduction for forward-only cars with a yaw-rate steering loop. This uses yaw-rate feedback, not a wheel-slip or sideslip-angle estimate. Vehicle testing is still required.

The receiver runs assistance locally from standard ELRS channel input and its onboard gyro. LetsFly and the configurator do not need to remain open. Saved settings are loaded on receiver startup; CH3 is forward throttle, CH4 is steering and the default activation switch is CH8. The zero-throttle endpoint follows the existing 988us mapping. Both ESC and steering outputs must use the compatible mixer described below.

## 使用

1. 在新版 GLRS 配置器的“飞控 → 漂移辅助”展开设置。已解锁时，CH3 油门归零（988μs）后可保存，无需关闭解锁开关或 CH7 共存开关；未解锁时也可保存。遥控端无法到达此值时，断开遥控链路、进入仅 Wi-Fi 模式后保存。
2. 保持 CH3 油门、CH4 偏航；油门使用正系数纯油门电机混控，转向使用纯偏航舵机混控，并将相应 PWM 输出设为混控来源。
3. 总开关默认关闭。启用后，默认由 CH8 的 `[1700, 2100)` μs 区间控制，仅 RATE 角速度模式下生效。开关不能占用已启用的解锁或 Wi-Fi 通道。
4. 默认介入偏差 20°/s、渐进区间 80°/s、最大减油 15%、介入时间常数 120ms、恢复时间常数 300ms。先架空核对停车值和方向，再低速试车。

The controller compares absolute measured yaw rate with the maximum absolute target in the previous 250 ms. The short window avoids excessive intervention when the stick crosses center during a reversal. Above the configured excess-rate threshold, the reduction rises linearly over the configured span and is smoothed with separate attack/recovery time constants. Each frame uses the original throttle; zero throttle stops immediately, and assistance never adds throttle above the manual request. Turning the remote switch off smoothly restores manual throttle. Disarm, RC loss/Wi-Fi-only operation, mode changes and configuration changes clear the history.

## Configuration and status

`GET/POST /config` adds `fc_drift`:

```json
{
  "enabled": false,
  "channel": 8,
  "range": [1700, 2100],
  "threshold_dps": 20,
  "ramp_dps": 80,
  "max_reduction_percent": 15,
  "attack_ms": 120,
  "recovery_ms": 300
}
```

Allowed ranges: channel 5–16; activation endpoints 900–2100 μs with start < end; threshold 5–100°/s; span 20–300°/s; reduction 0–30%; attack 50–500ms; recovery 100–1000ms. With the RC link connected, changes require valid recent state. Both explicit and automatic arming allow changes at zero CH3 throttle (988 μs, the existing control-loop endpoint), so the arm and CH7 coexistence switches can stay on. Armed nonzero throttle is rejected; disarmed changes remain allowed. This check uses the current receiver state, not the requested configuration. Unchanged drift settings and disconnected/Wi-Fi-only configuration remain allowed. Old configuration files default to assistance off. A missing field in a configuration request preserves the existing setting.

`GET /status.json` adds `drift` with the reason/state, remote request, target/measured yaw rates, manual/assisted throttle percentages and actual reduction percentage. Existing MSP packet layouts are unchanged.

## Validation

Run `pio test -e native -f test_drift` from `src`. The Native environment requires a host GCC compiler. On Windows, the installed PlatformIO toolchain can be added for the current terminal:

```powershell
$env:PATH = "$env:USERPROFILE\.platformio\packages\toolchain-gccmingw32\bin;" + $env:PATH
pio test -e native -f test_drift
```

Tests cover threshold, manual throttle priority, zero stop, symmetry, reduction limits, reset, time consistency, switch-off recovery, reversal handling, timer rollover, JSON validation and mixer compatibility.

The 409 save-guard fix additionally covers armed zero/nonzero throttle regardless of arming method, disarmed changes, unavailable state, unchanged settings and disconnected configuration. Hardware coexistence behavior still needs checking on the receiver.

Build `Unified_ESP32C3_2400_RX_via_WIFI` and bundle the `LightFin Nano 2.4GHz RX` hardware layout. A noninteractive build may produce a bare image; verify the hardware layout before flashing.

2026-10-12 validation: five drift tests passed. The full Windows Native run passed 57 cases; two existing MSP assertions failed, and CRC/OTA builds failed on the OTA8 struct-size assertion. All three affected suites reproduced the same failures on pristine upstream `4316c4aa` with the same MinGW toolchain. A pre-existing empty local `test_crash_detection` directory also reports “Nothing to build”. These are not a full-suite pass. No on-vehicle validation or flashing was performed.

2026-10-15 PR review: 56 selected Native regression cases passed, including all six drift cases; the known failing/empty suites above were excluded. ESP32-C3 firmware built and the LightFin hardware layout was packaged and verified. The JSON helper include is now inside the existing RX/flight-control compilation guard; a feature-disabled syntax check passed without ArduinoJson include paths. RF input, CRSF serialization and the existing flight-control scheduler remain unchanged. Vehicle behavior was not tested in this review.
