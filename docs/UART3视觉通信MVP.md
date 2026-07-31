# UART3视觉通信MVP

版本：视觉V1 / 底盘遥测V2

日期：2026-07-31

## 1. 接线与运行边界

| MSPM0G3519 | MaixCAM2 | 方向 |
| --- | --- | --- |
| PB12 / UART3 TX | A18 / UART4 RX | 56字节底盘遥测与低频文本 |
| PB13 / UART3 RX | A19 / UART4 TX | 24字节视觉状态 |
| GND | GND | 公共参考地 |

两端均为3.3 V TTL、115200-8-N-1，无硬件流控。UART3与I2C1红外循迹的
PA15/PA16及I2C0 OLED的PB17/PB18无引脚冲突。

通信联调固件关闭底盘自动循迹，且`main()`不初始化`emm42_demo_app`。解析到的视觉
状态仅通过快照接口提供，本轮不接入电机或Emm42控制。

## 2. 视觉V1固定24字节帧

| 偏移 | 长度 | 字段 | 定义 |
| ---: | ---: | --- | --- |
| 0 | 2 | `SOF` | `A5 5A` |
| 2 | 1 | `version` | `01` |
| 3 | 1 | `type` | `01`，钢球状态 |
| 4 | 1 | `frame_len` | 24 (`18`) |
| 5 | 1 | `flags` | bit0测量有效、bit1仅预测、bit2跟踪器就绪、bit3标定有效 |
| 6 | 2 | `sequence` | uint16小端，每帧递增并允许回绕 |
| 8 | 4 | `capture_ms` | Maix本地uint32毫秒时间，不与MCU时钟直接相减 |
| 12 | 2 | `position_dmm` | int16，0.1 mm/LSB |
| 14 | 2 | `velocity_mm_s` | int16，1 mm/s/LSB |
| 16 | 1 | `confidence` | 0..100 |
| 17 | 1 | `lost_frames` | 连续丢失帧数，255饱和 |
| 18 | 1 | `processing_ms` | 单帧处理耗时，255饱和 |
| 19 | 1 | `reserved` | 固定0 |
| 20 | 2 | `boot_id` | 视觉程序每次启动生成的新会话号 |
| 22 | 2 | `crc16` | 对0..21计算，低字节先发 |

`MEASURED_VALID`和`PREDICT_ONLY`不得同时置位。预测可携带定点值，但不会刷新有效
测量；丢失帧的位置和速度均为`INT16_MIN`。解析器接受的位置范围为
`-1300..+1300 dmm`，速度范围为`-5000..+5000 mm/s`。

CRC参数为`poly=0x1021, init=0xFFFF, refin=false, refout=false,
xorout=0x0000`。黄金帧如下，末尾小端CRC为`0xEFE4`：

```text
A5 5A 01 01 18 0D 34 12 E8 03 00 00 F4 01 88 FF 57 00 07 00 EF BE E4 EF
```

## 3. 解析、会话与失效

`vision_link_process()`在主循环中先于100 Hz底盘控制执行。解析器搜索帧头并处理粘包、
断帧和噪声；错误CRC、结构、语义、重复序号和反向序号均不更新快照。16位前向差值
小于`0x8000`时视为新帧，因此`FFFF -> 0000`正常回绕；序号跳变累计缺帧数。

`boot_id`变化时建立新会话，清除旧有效测量和“新测量待取”状态。接口分为：

- `vision_link_get_latest_snapshot()`：最近一帧接受的测量、预测或丢失状态；
- `vision_link_get_valid_measurement()`：仅在最近有效测量年龄不超过80 ms时返回；
- `vision_link_take_new_valid_measurement()`：每个有效测量最多交付一次；
- `vision_link_get_status()`：链路年龄、测量年龄、会话及全部错误计数。

最近一帧通过CRC且帧头受支持的视觉帧超过100 ms未到达时，`link_online=0`。UART0
每秒输出一次`[vision]`诊断。UART3 RX溢出由硬件环形缓冲计数并纳入状态。

## 4. 反向遥测与验收

MCU仍在底盘控制更新后发送100 Hz、56字节V2 `0x81`遥测，格式见
`UART3底盘遥测协议.md`。启动/存活文本使用`[link]`前缀，UART3不回显视觉字节。

验收顺序：黄金帧和故障注入、单向Maix TX、恢复全双工、停止/拔线/重启、1至8字节
插入删除、最后连续运行30分钟。通信联调全程要求底盘和Emm42不动作；硬件验收目标为
CRC误接受、UART溢出和遥测丢弃均为0，通信延迟99百分位小于5 ms。
