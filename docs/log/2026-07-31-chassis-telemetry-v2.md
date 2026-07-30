# 任务日志：UART3 底盘状态遥测 V2

| 字段 | 内容 |
| --- | --- |
| 提交 | `10264aa` feat: 更新 UART3 底盘状态遥测协议与相关功能 |
| 前置 | `0639022` V1 见 `2026-07-30-chassis-telemetry-v1.md` |
| 日期 | 2026-07-31 |
| 摘要 | 协议升至 **V2.0**：52→**56 字节**、10→**100 Hz**、新增 **IMU 加速度** 字段；与 100 Hz 轮速控制环同频 |

## 1. V1 → V2 差异总览

| 项 | V1 (`0639022`) | V2 (`10264aa`) |
| --- | --- | --- |
| version | `0x01` | `0x02` |
| frame_len | 52 (`0x34`) | 56 (`0x38`) |
| 帧头搜索 | `A5 5A 01 81 34` | `A5 5A 02 81 38` |
| 频率 | 10 Hz (100 ms) | 100 Hz (10 ms) |
| 线速占用 | ~4.5% @115200 | ~48.6% @115200 |
| 新增字段 | — | `imu_accel_x_mm_s2`、`imu_flags`、`imu_reserved` |
| CRC 范围 | 0..49 | 0..53 |

其余字段（轮速、PWM 拆分、循迹误差、tx_drop、rx_overflow）布局与 V1 兼容扩展。

## 2. 新增 IMU 字段

| 偏移 | 字段 | 说明 |
| ---: | --- | --- |
| 50 | `imu_accel_x_mm_s2` | int16，0.001 m/s²/LSB；无有效加速度时为 `INT16_MIN` |
| 52 | `imu_flags` | bit0=加速度有效（对应 `IMU_FLAG_ACCEL`） |
| 53 | `imu_reserved` | 固定 0 |

数据来源：`imu_get_snapshot()`，与 `wheel_speed_control` 同周期采样。

## 3. 代码变更

### `src/app/bluetooth_test_app.c`（后 `uart3_maix_app.c`）

- `CHASSIS_TELEMETRY_HZ`：10 → **100**。
- `CHASSIS_TELEMETRY_FRAME_SIZE`：52 → **56**。
- `CHASSIS_TELEMETRY_VERSION`：`0x01` → **`0x02`**。
- `chassis_telemetry_send()`：写入 IMU 加速度与 flags；CRC 覆盖至字节 53。

### `src/main.c`

- 确认 `uart3_maix_app_process()` 在 100 Hz 控制链中每圈调度（与 `motor_app_process` 同主循环）。

### `docs/UART3底盘遥测协议.md`

- 全文升级为 V2.0 规范；补充 100 Hz 下调至 50/20 Hz 的建议（当 `tx_drop_bytes` 持续增加）。

## 4. 编译开关

`control_config.h`：

```c
#define UART3_MAIX_CHASSIS_TELEMETRY_ENABLE   (0u)  /* 当前默认关 */
```

与摆杆遥测 `UART3_MAIX_BALANCE_TELEMETRY_ENABLE` **编译期互斥**（`80bd811` 引入）。

## 5. 验收要点

1. 搜帧头 `A5 5A 02 81 38`，56 字节 CRC 通过。
2. 100 Hz 下相邻 `mcu_ms` 差约 10 ms；`sequence` 单调递增。
3. IMU 就绪时 `imu_flags` bit0 置位，加速度非 `0x8000`。
4. 故意降低 TX 缓冲：整帧丢弃、`tx_drop_bytes` 递增、无半帧。

## 6. 下游工具（BallBalanceRover 根仓库）

- `SBDandT/recognition/tools/chassis_telemetry.py`：支持 V1/V2 解析（父仓库 `c2eaa1f`、`faf9eae`）。
- Maix 端 `uart_log_receiver.py` raw 模式录制后离线解码。

## 7. 当前运行模式

HEAD 默认 **`EMM42_BALANCE_DEMO_ENABLE=1`**，UART3 发送 **摆杆遥测 0x82**（20 B @100 Hz），底盘遥测 0x81 关闭。恢复底盘遥测需：

1. `EMM42_BALANCE_DEMO_ENABLE = 0`
2. `UART3_MAIX_CHASSIS_TELEMETRY_ENABLE = 1`
