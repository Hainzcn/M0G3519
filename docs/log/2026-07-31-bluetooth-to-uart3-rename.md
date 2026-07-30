# 任务日志：bluetooth → uart3_maix 命名重构与公式文档

| 字段 | 内容 |
| --- | --- |
| 提交 | `41a45c7` docs: 更新文档 |
| 日期 | 2026-07-31 |
| 摘要 | 全局重命名 **bluetooth_*** → **uart3_maix_***；补充连杆运动学公式；压缩连杆示意图；任务日志文件重命名 |

## 1. 动机

UART3 链路已承担 Maix 视觉 + 底盘/摆杆遥测，继续称为 “bluetooth” 易与 UART 蓝牙模块混淆。本提交做**纯命名与文档整理**，不改变协议语义。

## 2. 文件重命名映射

| 原路径 | 新路径 |
| --- | --- |
| `src/app/bluetooth_test_app.c/h` | `src/app/uart3_maix_app.c/h` |
| `src/hardware/bluetooth_hw.c/h` | `src/hardware/uart3_maix_hw.c/h` |
| `docs/log/2026-07-30-bluetooth-uart-test-mode.md` | `docs/log/2026-07-30-uart3-maix-test-mode.md` |

- 删除旧 `bluetooth_test_app.h` 等 7 个文件，新增对等 `uart3_maix_*`（内容迁移 + 符号替换）。
- `main.c`：`#include` 与 init/process 调用同步更新。

## 3. 代码微调

### `uart3_maix_app.c`

- 启动/存活字符串统一 **`[link]`** 前缀（原 `[bt]`）。
- 摆杆/底盘遥测分支提示（为 `80bd811` 预埋）。

### `vision_link.c`

- `#include "uart3_maix_hw.h"`（原 bluetooth_hw）。

### `control_config.h`

- 补充遥测互斥与 balance demo 相关宏注释。

## 4. 文档更新

### `docs/formula.md`（+207 行量级改写）

- 补充 **四连杆摆杆逆解** 公式与符号定义（CB、DX、DY、DP、BP、α、θ）。
- 与 `emm42_demo_app.c` 中 `emm42_demo_linkage_inverse()` 对齐。
- 区分摆杆角 α 与电机曲柄角 θ。

### `docs/H题_详细设计技术路线.md`

- 视觉/遥测章节指向 `UART3视觉通信MVP.md` 与 `UART3底盘遥测协议.md` 为权威来源。

### `docs/images/连杆.png`

- 体积 311 KB → 93 KB（无损压缩/重导出），便于文档加载。

## 5. 工程配置

- `keil/.eide/eide.yml`：源文件列表替换为新命名。

## 6. 行为变化

**无功能性协议变更**。若外部脚本仍搜索 `[bt] alive`，需改为 `[link] alive`。

## 7. 文件变更统计

| 类别 | 数量 |
| --- | --- |
| 重命名/迁移 | 4 对 .c/.h |
| 删除 bluetooth 旧文件 | 4 |
| 文档 | formula、技术路线、连杆图 |
| 净变更 | +466 / -352（git stat） |
