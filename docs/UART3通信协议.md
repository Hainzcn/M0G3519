# MaixCAM2 与 MSPM0G3519 UART3 通信协议

版本：视觉 V1 / 运行遥测 V1 / 调试遥测兼容

日期：2026-08-05

## 1. 接线和运行模式

| MSPM0G3519 | MaixCAM2 | 正常模式方向 |
| --- | --- | --- |
| PB13 / UART3 RX | A19 / UART4 TX | Maix 视觉状态下发 |
| PB12 / UART3 TX | A18 / UART4 RX | MCU 运行遥测上报 |
| GND | GND | 公共参考地 |

两端均为 3.3V TTL、115200-8-N-1、无硬件流控。统一帧包络为
`A5 5A + version + type + frame_len + payload + CRC16`，多字节整数均按小端发送，
CRC 使用 CRC-16/CCITT-FALSE：`poly=0x1021, init=0xFFFF, xorout=0`。

`0x83` 运行遥测不受 `UART3_MAIX_MODE` 影响，上电后始终以 100Hz 发送。
`control_config.h` 的模式选择只控制是否附加旧调试遥测：

| 模式 | UART3 RX | UART3 TX | 用途 |
| --- | --- | --- | --- |
| `UART3_MAIX_MODE_NORMAL` | 视觉 V1 | `0x83` V1，100Hz | 正常工作，默认值 |
| `UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG` | 视觉 V1 | `0x83` + `0x81` V2 | 兼容底盘台架调试 |
| `UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG` | 视觉 V1 | `0x83` + `0x82` | 兼容摆杆调试 |

UART3 不发送启动、存活或诊断文本；所有人类可读诊断只走 UART0。

### 1.1 运行遥测 V1 固定 24 字节帧

该帧只保留运行采集需要的按键、前向加速度和编码器转速。MaixCAM2 只需原样保存
UART 字节流，不需要在线解析。

| 偏移 | 长度 | 字段 | 定义 |
| ---: | ---: | --- | --- |
| 0 | 2 | `SOF` | `A5 5A` |
| 2 | 1 | `version` | `01` |
| 3 | 1 | `type` | `83`，运行遥测 |
| 4 | 1 | `frame_len` | 24 (`18`) |
| 5 | 1 | `flags` | bit0：IMU 加速度有效且新鲜 |
| 6 | 1 | `last_button` | 最近一次按下的键：0 无、1..4 对应 SW1..SW4 |
| 7 | 1 | `active_button` | 当前仍按住的键：0 无、1..4 对应 SW1..SW4 |
| 8 | 2 | `button_sequence` | 每次新按键沿递增，uint16 回绕 |
| 10 | 2 | `sequence` | 每帧递增，uint16 回绕 |
| 12 | 4 | `mcu_ms` | MCU 上电毫秒计时，uint32 回绕 |
| 16 | 2 | `forward_accel_mm_s2` | int16，车头正方向 mm/s²；无效为 `INT16_MIN` |
| 18 | 2 | `left_encoder_rpm_x10` | int16，左轮前进为正，0.1RPM/LSB |
| 20 | 2 | `right_encoder_rpm_x10` | int16，右轮前进为正，0.1RPM/LSB |
| 22 | 2 | `crc16` | 对 0..21 计算，低字节先发 |

`last_button` 与 `button_sequence` 在松键后继续保留，便于仅保存原始流时可靠定位短按；
`active_button` 用于区分按住和已松开。IMU X 轴按
`BALANCE_SIMPLE_CAR_ACCEL_SIGN` 统一到车头正方向，不扣静态零偏。

## 2. 视觉 V1 固定 24 字节帧

| 偏移 | 长度 | 字段 | 定义 |
| ---: | ---: | --- | --- |
| 0 | 2 | `SOF` | `A5 5A` |
| 2 | 1 | `version` | `01` |
| 3 | 1 | `type` | `01`，钢球状态 |
| 4 | 1 | `frame_len` | 24 (`18`) |
| 5 | 1 | `flags` | bit0 测量有效，bit1 仅预测，bit2 跟踪器就绪，bit3 标定有效 |
| 6 | 2 | `sequence` | uint16，每个新图像递增并允许回绕 |
| 8 | 4 | `capture_ms` | Maix 本地 uint32 毫秒时间，不与 MCU 时钟直接相减 |
| 12 | 2 | `position_dmm` | int16，0.1mm/LSB |
| 14 | 2 | `velocity_mm_s` | int16，1mm/s/LSB |
| 16 | 1 | `confidence` | 0..100 |
| 17 | 1 | `lost_frames` | 连续丢失图像数，255 饱和 |
| 18 | 1 | `processing_ms` | 单帧处理耗时，255 饱和 |
| 19 | 1 | `reserved` | 固定 0 |
| 20 | 2 | `boot_id` | Maix 视觉程序每次启动生成的新会话号 |
| 22 | 2 | `crc16` | 对 0..21 计算，低字节先发 |

bit0 与 bit1 不得同时置位。预测可以携带数值，但不刷新主控有效测量；LOST、非有限
数值或越界状态的位置与速度均为 `INT16_MIN`。协议范围为位置 -130.0..+130.0mm、
速度 -5000..+5000mm/s。

黄金帧为有效测量、序号 `0x1234`、采集时间 1000ms、位置 +50.0mm、速度
-120mm/s、置信度 87、处理耗时 7ms、会话号 `0xBEEF`：

```text
A5 5A 01 01 18 0D 34 12 E8 03 00 00 F4 01 88 FF 57 00 07 00 EF BE E4 EF
```

## 3. 主控解析和失效语义

UART RX 中断只写 256 字节环形缓冲，`vision_link_process()` 在主循环中搜索帧头、
检查版本/类型/长度、CRC、保留位、状态位和物理范围。CRC 或语义错误、重复序号和
反向序号不更新测量快照；`FFFF -> 0000` 是正常回绕。

- `boot_id` 改变：建立新会话并清除旧测量和序号历史；
- 最近真实测量超过 80ms：`measurement_valid=0`；
- 最近支持且 CRC 正确的视觉帧超过 100ms：`link_online=0`；
- `vision_link_take_new_valid_measurement()` 每个真实测量最多交付一次；
- MCU 使用本地 `received_ms` 判断年龄，不直接比较 Maix 的 `capture_ms`。

V1 不定义 ACK、重传、控制命令、自动波特率或跨芯片时钟同步。

## 4. 两种 100Hz 调试遥测

底盘调试帧为 `A5 5A 02 81 38`、固定 56 字节，字段见
`UART3底盘遥测协议.md`。

运动 Demo 帧为 `A5 5A 02 82 16`、固定 22 字节：

| 偏移 | 长度 | 字段 | 定义 |
| ---: | ---: | --- | --- |
| 5 | 1 | `flags` | bit0 Demo 活跃，bit1 IMU 有效，bit2 电机位置有效 |
| 6 | 1 | `demo_state` | 最低边界置零、回水平和正负 5 度往复状态 |
| 7 | 1 | `reserved` | 固定 0 |
| 8 | 2 | `sequence` | uint16，100Hz 递增 |
| 10 | 4 | `mcu_ms` | MCU 本地毫秒时间 |
| 14 | 2 | `target_angle_cdeg` | 目标摆杆角，0.01deg/LSB |
| 16 | 2 | `imu_pitch_cdeg` | IMU 原始 pitch，0.01deg/LSB；无效为 `INT16_MIN` |
| 18 | 2 | `motor_feedback_cdeg` | Emm42 当前电机轴角，0.01deg/LSB；无效为 `INT16_MIN` |
| 20 | 2 | `crc16` | 对 0..19 计算 |

闭环控制帧为 `A5 5A 02 82 28`、固定 40 字节：

| 偏移 | 长度 | 字段 | 定义 |
| ---: | ---: | --- | --- |
| 5 | 1 | `flags` | bit0 ACTIVE，bit1 电机反馈有效，bit2 视觉在线，bit3 已接受测量，bit4 命令等待，bit5 故障锁存 |
| 6 | 1 | `balance_state` | 摆杆状态机 |
| 7 | 1 | `fault` | 锁存故障码，0 表示无故障 |
| 8 | 2 | `sequence` | uint16，100Hz 递增 |
| 10 | 4 | `mcu_ms` | MCU 本地毫秒时间 |
| 14 | 2 | `vision_sequence` | 最近接受的视觉帧序号 |
| 16 | 2 | `position_dmm` | 估计球位置，0.1mm/LSB |
| 18 | 2 | `velocity_mm_s` | 估计球速度，1mm/s/LSB |
| 20 | 2 | `error_dmm` | 中心目标位置误差，0.1mm/LSB |
| 22 | 2 | `desired_accel_mm_s2` | 期望球加速度，1mm/s^2/LSB |
| 24 | 2 | `lever_angle_cdeg` | 摆杆目标角，0.01deg/LSB |
| 26 | 2 | `motor_target_cdeg` | Emm42 绝对目标角，0.01deg/LSB |
| 28 | 2 | `motor_feedback_cdeg` | Emm42 `0x36` 当前电机轴角，10Hz 查询并缓存；无效为 `INT16_MIN` |
| 30 | 2 | `vision_age_ms` | 有效测量年龄；无效为 `UINT16_MAX` |
| 32 | 1 | `confidence` | 最近接受测量的置信度 |
| 33 | 1 | `control_flags` | 测量新鲜、预测、边缘恢复及三类限幅标志 |
| 34 | 2 | `command_errors` | Emm42 命令错误累计值 |
| 36 | 2 | `emm42_rx_overflows` | UART7 RX 溢出累计值，饱和到 65535 |
| 38 | 2 | `crc16` | 对 0..37 计算 |

MaixCAM2 在这两种模式下运行 `uart_log_receiver.py`，接收器原样轮转保存 `.bin`。
摆杆日志由仓库 `tools/balance_log/analyze_balance_log.py` 离线生成 CSV 和调参曲线；
原始文件始终是故障分析的权威数据。

除故障锁存状态外，`balance_app` 每 100ms 发起一次 Emm42 `0x36` 当前位置查询；
未完成启动标定时仅执行这一只读查询，不使能或移动电机。闭环状态收到位置响应后，
仍可按独立的 100ms 运动指令周期发送到期的绝对位置目标。遥测以 100Hz 发送最近收到的
当前位置，bit1 表示电机反馈已经有效。

回水平完成后状态机先进入 `WAIT_VISION` 并保持水平。连续 5 个完整合格状态帧后才
置 ACTIVE；预测帧、低置信度帧或缺少跟踪器/标定有效位的帧都会清零连续计数。
UART0 的 `[balance]` 诊断输出原始 flags、置信度、位置以及接受状态，避免将链路层
`[vision] valid=1` 误判为控制层已经接受测量。

## 5. 联调顺序

1. 正常模式先发送黄金帧，不连接执行器，确认主控 UART0 诊断无 CRC/语义错误。
2. 运行真实视觉发送，确认序号、`boot_id`、位置和速度符合物理方向。
3. 注入错误 CRC、插入/删除 1..8 字节、重启和拔线，验证重同步与 80/100ms 失效。
4. 分别编译两个遥测调试模式，Maix 独立采集 30 分钟，确认 CRC、序号缺口、UART
   溢出和主控 TX 整帧丢弃计数均为 0。
5. 验收结束后恢复 `UART3_MAIX_MODE_NORMAL`，确认 PB12 在正常工作中无报文。
