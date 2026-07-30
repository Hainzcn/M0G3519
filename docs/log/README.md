# M0G3519 任务日志索引（8929637 → HEAD）

起始提交 **`8929637`**（Emm42 步进驱动）至 **`80bd811`**（摆杆 Demo + 按键）期间的任务日志汇总。  
仓库路径：`M0G3519/`（MSPM0G3519 固件子模块）。

> 注：用户所称 M0G3507 对应本仓库 **MSPM0G3519** 子项目；起始 hash 在该子模块 git 历史中有效。

## 提交时间线

| 日期 | 提交 | 摘要 | 任务日志 |
| --- | --- | --- | --- |
| 07-30 | `8929637` | Emm42 UART7 驱动与 Demo 骨架 | [2026-07-30-emm42-stepper-driver.md](2026-07-30-emm42-stepper-driver.md) |
| 07-30 | `01a14c7` | UART3 蓝牙回显测试模式 | [2026-07-30-uart3-maix-test-mode.md](2026-07-30-uart3-maix-test-mode.md) |
| 07-30 | `1597240` | Maix 通信文档废止旧草案 | ↑ 并入 [chassis-telemetry-v1](2026-07-30-chassis-telemetry-v1.md) |
| 07-30 | `0639022` | UART3 底盘遥测 **V1**（52 B @10 Hz） | [2026-07-30-chassis-telemetry-v1.md](2026-07-30-chassis-telemetry-v1.md) |
| 07-30 | `8355960` | 蓝牙应用与循迹联调增强 | ↑ 同上 |
| 07-31 | `0f2ec26` | 循迹 **查表+相位** 重构 | [2026-07-31-line-control-lookup-refactor.md](2026-07-31-line-control-lookup-refactor.md) |
| 07-31 | `10264aa` | 底盘遥测 **V2**（56 B @100 Hz + IMU） | [2026-07-31-chassis-telemetry-v2.md](2026-07-31-chassis-telemetry-v2.md) |
| 07-31 | `e9d22af` | 红外循迹 **I2C1** 重构 + i2c_bus | [2026-07-31-grayscale-i2c-refactor.md](2026-07-31-grayscale-i2c-refactor.md) |
| 07-31 | `a026f4b` | Maix **视觉链路 MVP**（24 B RX） | [2026-07-31-vision-link-mvp.md](2026-07-31-vision-link-mvp.md) |
| 07-31 | `41a45c7` | bluetooth→uart3_maix 重命名 + formula | [2026-07-31-bluetooth-to-uart3-rename.md](2026-07-31-bluetooth-to-uart3-rename.md) |
| 07-31 | `80bd811` | 摆杆 ±5° Demo + 遥测 0x82 + 按键 | [2026-07-31-balance-pendulum-demo-and-button.md](2026-07-31-balance-pendulum-demo-and-button.md) |

当前子模块 HEAD：`80bd811`（BallBalanceRover 父仓库可能指向 `e9d22af` 或更新，以 `.gitmodules` / `git submodule status` 为准）。

## 按子系统归类

### UART3 / Maix 通信

| 方向 | 协议 | 文档 |
| --- | --- | --- |
| Maix → MCU | 视觉 V1，24 B，`0x01` | `docs/UART3视觉通信MVP.md` |
| MCU → Maix | 底盘遥测 V2，56 B，`0x81` @100 Hz | `docs/UART3底盘遥测协议.md` |
| MCU → Maix | 摆杆遥测 V1，20 B，`0x82` @100 Hz | `docs/UART3视觉通信MVP.md` §4 |

代码入口：`uart3_maix_app.c`、`uart3_maix_hw.c`、`vision_link.c`。

### 底盘控制

| 模块 | 文件 | 日志 |
| --- | --- | --- |
| 循迹外环（查表） | `line_control.c` | line-control-lookup-refactor |
| 轮速内环 | `wheel_speed_control.c` | chassis-telemetry-v1 |
| 公共配置 | `control_config.h` | 各日志均有提及 |

### 感知与外设

| 模块 | 文件 | 日志 |
| --- | --- | --- |
| 灰度 I2C | `i2c_bus.c`、`grayscale*.c` | grayscale-i2c-refactor |
| OLED | `oled_hw.c`、`oled_app.c` | ↑ 同上 |
| 按键 | `button*.c` | balance-pendulum-demo-and-button |
| Emm42 摆杆 | `emm42*.c`、`emm42_demo_app.c` | emm42-stepper-driver、balance-pendulum |

### 权威协议 / API 文档（非 log）

- `docs/UART3视觉通信MVP.md`
- `docs/UART3底盘遥测协议.md`
- `docs/Emm42步进电机驱动.md`
- `docs/红外循迹模块适配.md`
- `docs/循迹两驱PID框架.md`
- `docs/formula.md`
- `docs/api/README.md`

## 当前固件默认行为（HEAD）

1. 底盘 **不**自动循迹（`MOTOR_APP_AUTO_START_* = 0`）。
2. **摆杆 Demo 启用**（`EMM42_BALANCE_DEMO_ENABLE = 1`）：±5° 往复 + UART3 发 0x82。
3. UART3 RX：24 B 视觉解析；1 Hz `[link] alive` + UART0 `[vision]` 诊断。
4. 红外循迹 I2C 2 ms 轮询；OLED 显示灰度/按键/错误。
5. 底盘遥测 0x81 **关闭**（与 0x82 互斥）。

## 更早日志

| 文件 | 范围 |
| --- | --- |
| [2026-07-29-chassis-control-and-peripheral-refactor.md](2026-07-29-chassis-control-and-peripheral-refactor.md) | 双层 PID 框架、IMU DMA、OLED 重写 |
| [2026-07-28-oled-display-module.md](2026-07-28-oled-display-module.md) | OLED 首次接入 |
| [2026-07-27-*.md](.) | 灰度/IMU/编码器 |
