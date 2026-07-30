# 任务日志：UART3 底盘状态遥测 V1

日期：2026-07-30  
提交：`0639022`、`8355960`（部分）、`1597240`（协议文档废止旧草案）

## 背景

MaixCAM2 需要接收 MCU 侧底盘运行状态以做离线分析与闭环调试。在蓝牙 UART 测试模式稳定后，于同一 UART3 链路上增加 MCU→Maix 的二进制遥测帧，与后续 Maix→MCU 视觉帧形成全双工基础。

## 提交 `0639022`：V1 协议与发送实现

### 新增文档

- `docs/UART3底盘遥测协议.md`（V1.0）：52 字节固定帧、`0x81` 类型、10 Hz、CRC-16/CCITT-FALSE。

### 协议要点（V1）

| 项 | 值 |
| --- | --- |
| 物理层 | UART3 PB12 TX / PB13 RX，115200-8-N-1 |
| 方向 | MCU → MaixCAM2 |
| 频率 | 10 Hz（100 ms） |
| 帧长 | 52 字节 |
| 搜索头 | `A5 5A 01 81 34` |

载荷覆盖：`vehicle_state`、`flags`、序号、`mcu_ms`、规划/实测轮速与前馈/反馈 PWM、循迹误差、`tx_drop_bytes`、`rx_overflow_count`。

### 代码变更（按模块）

**`src/app/bluetooth_test_app.c`（后改名为 `uart3_maix_app.c`）**

- 新增 `chassis_telemetry_send()`：组装 52 字节帧并经 TX 环形缓冲原子入队。
- 100 ms 周期调度；整帧空间不足时丢弃并累计 drop 计数。
- 启动/存活 ASCII 文本保留（`[link]` 前缀在后续提交统一）。

**`src/middle/wheel_speed_control.c/h`**

- 新增 `wheel_speed_control_status_t` 快照结构。
- 暴露规划速度/加速度、左右目标/实测 RPM、前馈/反馈/最终 PWM、饱和与运动学有效标志。
- 供遥测帧直接读取，避免重复计算。

**`src/middle/control_pid.c/h`**

- PID 输出拆分前馈项与反馈项，便于遥测区分 kS/kV/kA 与积分/微分贡献。

**`src/hardware/bluetooth_hw.c/h`**

- 增加 `bluetooth_hw_get_rx_overflow_count()`、`bluetooth_hw_write_atomic()` 等遥测所需接口。

**`src/main.c`**

- 集成遥测发送调度；联调阶段保持底盘停机（`MOTOR_APP_AUTO_START_* = 0`）。

**`src/middle/control_config.h`**

- 新增 `UART3_MAIX_CHASSIS_TELEMETRY_ENABLE` 编译开关。

## 提交 `8355960`：与循迹联调增强

- `main.c` 挂载完整应用链（IMU、灰度、电机等），遥测在真实控制环下验证。
- `control_config.h`：调整轮距、PID 增益、曲率前馈相关宏。
- `line_control.c`：小幅适配，为后续查表重构铺垫。
- 文档补充双轮 PID 框架说明。

## 提交 `1597240`：Maix 通信文档整理

- `docs/H题_详细设计技术路线.md`：废止旧版视觉物理层/钢球测量帧草案，明确以独立协议文档为准；默认波特率 115200，230400 为可选升级。

## 验收要点（V1）

1. Maix 或 PC 端按 `A5 5A 01 81 34` 搜帧，CRC 通过后解析字段。
2. 10 Hz 下 `sequence` 单调递增（允许 uint16 回绕）。
3. 底盘停机关闭控制时 `vehicle_state=DISABLED`，`CONTROL_ACTIVE` 位清零。
4. 故意塞满 TX 缓冲时 `tx_drop_bytes` 递增且无半帧泄漏。

## 后续

- `0f2ec26`：循迹查表重构，V1 帧中 `line_error` 语义随查表等级变化。
- `10264aa`：协议升级 V2.0（56 字节、100 Hz、IMU 加速度字段），见 `2026-07-31-chassis-telemetry-v2.md`。
