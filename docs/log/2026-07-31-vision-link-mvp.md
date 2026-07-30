# 任务日志：Maix 视觉链路解析 MVP

| 字段 | 内容 |
| --- | --- |
| 提交 | `a026f4b` feat: 实现视觉链路解析MVP（作者 waang1） |
| 日期 | 2026-07-31 |
| 摘要 | 新增 **24 字节视觉 V1** 固定帧解析器 `vision_link`；UART3 RX 由字节回显改为状态机收帧；每秒 UART0 输出 `[vision]` 诊断 |

## 1. 协议概要（Maix → MCU）

权威文档：`docs/UART3视觉通信MVP.md` §2。

| 项 | 值 |
| --- | --- |
| SOF | `A5 5A` |
| version / type | `01` / `01`（钢球状态） |
| frame_len | 24 |
| CRC | CCITT-FALSE，覆盖 0..21 |
| 黄金帧 CRC | `EFE4`（文档内完整样例） |

关键载荷：`sequence`、`capture_ms`、`position_dmm`（0.1 mm）、`velocity_mm_s`、`confidence`、`lost_frames`、`boot_id`。

## 2. 新增 `src/middle/vision_link.c/h`（~445 行）

### 2.1 接收状态机

- 在 `vision_link_process()` 中从 `uart3_maix_hw` 逐字节读取。
- 搜 SOF → 收满 24 B → CRC → 语义校验。
- 处理粘包、断帧、噪声；错误帧不更新快照。

### 2.2 语义规则

- `MEASURED_VALID` 与 `PREDICT_ONLY` 互斥。
- 位置 `-1300..+1300 dmm`，速度 `-5000..+5000 mm/s`。
- 序号前进差 < `0x8000` 视为新帧；`FFFF→0000` 正常回绕。
- `boot_id` 变化 → 新会话，清除旧有效测量。

### 2.3 对外 API

| 函数 | 用途 |
| --- | --- |
| `vision_link_get_latest_snapshot()` | 最近接受的帧（含预测/丢失） |
| `vision_link_get_valid_measurement()` | 有效测量且 age ≤ 80 ms |
| `vision_link_take_new_valid_measurement()` | 单次交付，防重复消费 |
| `vision_link_get_status()` | 链路/测量年龄、错误计数、溢出 |

### 2.4 失效判定

- 100 ms 无 CRC 有效视觉帧 → `link_online=0`。
- UART3 RX 溢出计入 `vision_link_status_t.uart_rx_overflows`。

### 2.5 并发安全

- 双 generation 计数器实现快照无锁读（主循环读 / ISR 写分离）。

## 3. 应用层集成

### `src/app/bluetooth_test_app.c`

- `init()` 调用 `vision_link_init()`。
- **删除 RX 字节回显**；改为 `vision_link_process()`。
- 1 Hz：`[link] alive` + `vision_link_send_diagnostic()` 经 **UART0** 输出 `[vision] on=…` 统计。

### `src/main.c`

- 主循环顺序：`vision_link_process()` **先于** `motor_app_process()`（保证控制环读到最新测量）。

### `src/middle/control_config.h`

- 联调阶段保持底盘停机宏不变。

## 4. 文档

- 新增 `docs/UART3视觉通信MVP.md`（视觉 V1 + 链路边界）。
- 更新 `docs/UART3底盘遥测协议.md`：注明 MVP 已删除 RX 回显。
- `docs/log/2026-07-30-bluetooth-uart-test-mode.md` 加注废止说明。

## 5. 工程

- `keil/.eide/eide.yml`：纳入 `vision_link.c` 编译项。

## 6. Maix 端（SBDandT 子模块，父仓库）

| 提交 | 内容 |
| --- | --- |
| `0f34238` | Maix UART 改 UART4，文档同步 |
| `a2e122a` | `uart_log_receiver.py` 增强录制 |
| `b0e7dac` | 主控 UART 通信 MVP 接入 |

Maix 发送 24 B 视觉帧；MCU 回传遥测（底盘 0x81 或摆杆 0x82，取决于编译开关）。

## 7. 验收顺序（文档 §5）

1. 黄金帧 + 故障注入（CRC/序号/语义）。
2. 单向 Maix TX → 恢复全双工。
3. 停止/拔线/重启、`boot_id` 变化后会话重建。
4. 1–8 字节随机插入/删除压力。
5. 连续运行 30 min：`crc_errors=0`、无 RX 溢出。

## 8. 后续

- `41a45c7`：`bluetooth_*` → `uart3_maix_*` 命名统一。
- `80bd811`：增加 MCU→Maix **摆杆遥测 0x82**（20 B @100 Hz）。
