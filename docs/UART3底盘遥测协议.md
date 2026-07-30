# UART3 底盘状态遥测协议

版本：V1.0

日期：2026-07-30

## 1. 链路与发送规则

- 物理接口：MSPM0G3519 UART3，PB12 TX、PB13 RX，3.3 V TTL；
- 串口参数：115200-8-N-1，无硬件流控；
- 方向：MCU -> MaixCAM2；
- 频率：10 Hz，每 100 ms 发送一帧；
- 帧长：固定 52 字节；
- 多字节整数：小端序；
- 完整性：CRC-16/CCITT-FALSE；
- 消息类型：`0x81 CHASSIS_TELEMETRY`。

UART3 继续允许低频 ASCII 启动/存活信息和调试回显。接收端应搜索
`A5 5A 01 81 34`，按长度收齐后校验 CRC，不得按换行拆分整个 UART 字节流。

遥测帧通过 256 字节非阻塞 TX 环形缓冲发送。整帧入队是原子的：剩余空间不足时
丢弃整帧并累计 `tx_drop_bytes`，不会把半帧留在串口流中。10 Hz 遥测线速为：

```text
52 byte/frame * 10 bit/byte * 10 frame/s = 5200 bit/s
```

占 115200 bit/s 的约 4.5%。

## 2. 固定 52 字节布局

| 偏移 | 长度 | 字段 | 类型/单位 | 说明 |
| ---: | ---: | --- | --- | --- |
| 0 | 2 | `SOF` | uint8[2] | 固定 `A5 5A` |
| 2 | 1 | `version` | uint8 | 固定 `0x01` |
| 3 | 1 | `type` | uint8 | 固定 `0x81` |
| 4 | 1 | `frame_len` | uint8 | 固定 52 (`0x34`) |
| 5 | 1 | `vehicle_state` | uint8 | `motor_app_mode_enum` |
| 6 | 1 | `flags` | uint8 | 状态位，见第 3 节 |
| 7 | 1 | `reserved` | uint8 | V1 固定为 0 |
| 8 | 2 | `sequence` | uint16 | 每个计划样本递增，允许回绕 |
| 10 | 4 | `mcu_ms` | uint32 ms | MCU 本地毫秒时间 |
| 14 | 2 | `planned_speed_mm_s` | int16 mm/s | 左右目标 RPM 均值换算 |
| 16 | 2 | `planned_accel_mm_s2` | int16 mm/s² | 规划速度在 100 Hz 下的差分 |
| 18 | 2 | `left_target_rpm_x10` | int16 0.1 RPM | 左轮目标转速 |
| 20 | 2 | `right_target_rpm_x10` | int16 0.1 RPM | 右轮目标转速 |
| 22 | 2 | `left_measured_rpm_x10` | int16 0.1 RPM | 左编码器实测转速 |
| 24 | 2 | `right_measured_rpm_x10` | int16 0.1 RPM | 右编码器实测转速 |
| 26 | 2 | `left_feedforward_pwm` | int16 duty | 左轮 kS/kV/kA 前馈，经左右映射 |
| 28 | 2 | `right_feedforward_pwm` | int16 duty | 右轮 kS/kV/kA 前馈，经左右映射 |
| 30 | 2 | `left_feedback_pwm` | int16 duty | 左轮 PID 反馈项，经左右映射 |
| 32 | 2 | `right_feedback_pwm` | int16 duty | 右轮 PID 反馈项，经左右映射 |
| 34 | 2 | `left_final_pwm` | int16 duty | 限幅、映射、斜率限制后的逻辑命令 |
| 36 | 2 | `right_final_pwm` | int16 duty | 限幅、映射、斜率限制后的逻辑命令 |
| 38 | 2 | `measured_speed_mm_s` | int16 mm/s | 左右实测 RPM 均值换算 |
| 40 | 2 | `measured_accel_mm_s2` | int16 mm/s² | 编码器速度差分的滤波值 |
| 42 | 2 | `line_error` | int16 | 查表位置等级乘以 1000，范围 -4000..4000 |
| 44 | 4 | `tx_drop_bytes` | uint32 | UART3 TX 缓冲累计丢弃字节数 |
| 48 | 2 | `rx_overflow_count` | uint16 | UART3 RX 溢出次数，65535 饱和 |
| 50 | 2 | `crc16` | uint16 | 对字节 0..49 计算，低字节先发 |

超出 int16 表示范围的连续量发送饱和值，不发生整数回绕。

## 3. 状态定义

`vehicle_state`：

| 值 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | `DISABLED` | 控制停用，最终 PWM 为 0 |
| 1 | `SPEED_TEST` | 左右轮速度测试 |
| 2 | `RIGHT_CIRCLE_DEMO` | 顺时针圆周测试 |
| 3 | `LINE_FOLLOW` | 灰度循迹 |

`flags`：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | `CONTROL_ACTIVE` | 当前不是 `DISABLED` |
| 1 | `LINE_VALID` | 当前扫描得到有效循迹误差 |
| 2 | `LINE_LOST` | 循迹线已丢失 |
| 3 | `MARKER` | 检测到横向标志线 |
| 4 | `LEFT_SATURATED` | 左轮 PID 输出饱和 |
| 5 | `RIGHT_SATURATED` | 右轮 PID 输出饱和 |
| 6 | `KINEMATICS_VALID` | 速度/加速度估计已初始化 |
| 7 | `UART_RX_OVERFLOW` | UART3 曾发生 RX 缓冲溢出 |

## 4. 速度、加速度与 PWM 语义

车辆规划速度和实测速度均由左右轮平均转速换算：

```text
v = (rpm_left + rpm_right) / 2 * pi * wheel_diameter / 60
```

当前“规划器”是循迹目标 RPM 的斜率限制器，并非完整 S 曲线。因此
`planned_accel_mm_s2` 表示斜率限制后的目标速度差分。后续引入 S 曲线规划器时，
该字段应直接使用规划器输出，字段含义和比例不变。

`measured_accel_mm_s2` 是编码器速度在 10 ms 控制周期内差分后的估计值：原始值先
限幅到 +/-20 m/s²，再使用 `alpha=0.20` 的一阶低通。它适合分析轮速响应和提供底盘
纵向前馈参考，但不等同于 IMU 的车体轴线性加速度；IMU 原始加速度在使用前还需要
安装轴确认、零偏标定和重力补偿。

PWM 三部分含义：

```text
feedforward = kS*sign(target) + kV*target_rpm + kA*target_accel
feedback    = Kp*error + Ki*integral + Kd*error_derivative
final       = clamp(feedforward + feedback) -> wheel map -> slew limit
```

由于存在总输出限幅和斜率限制，`feedforward_pwm + feedback_pwm` 不保证在每个周期
严格等于 `final_pwm`。这三个字段用于区分模型前馈、闭环修正和真实执行命令。
`final_pwm` 是进入 `motor_set_speed()` 前、按车辆左右轮定义的逻辑命令；底层为适配
接线而进行的电机极性翻转和物理通道交换不改变该字段。

## 5. CRC 与接收检查

CRC 参数：

```text
poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
```

以下黄金帧表示：车辆停用、循迹线丢失、运动学估计已初始化，其余字段和计数均为
0。末尾小端 CRC 为 `0x2EE8`：

```text
A5 5A 01 81 34 00 44 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 E8 2E
```

接收端处理顺序：

1. 搜索 `A5 5A`；
2. 检查版本 `01`、类型 `81`、长度 `34`；
3. 收齐 52 字节并对 0..49 计算 CRC；
4. CRC 正确后按小端定点字段解析；
5. 用 `sequence` 统计丢帧，用 `mcu_ms` 计算 MCU 侧采样间隔；
6. CRC 错误时丢弃当前第一个帧头字节，继续搜索下一帧。

MaixCAM2 的 `uart_log_receiver.py` 会原样保存包含文本和二进制帧的整个流，因此原始
`.bin` 文件始终是离线分析的权威数据源。
