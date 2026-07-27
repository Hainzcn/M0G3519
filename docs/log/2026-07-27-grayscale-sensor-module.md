# 任务日志：八路循迹模块接入与心跳计数修复

| 字段 | 内容 |
| --- | --- |
| 提交 | `f40006d10a6be10679c3c738886373df20a21482` — feat: 添加灰度传感器模块及相关功能 |
| 父提交 | `b8ba0f6` — feat: 更新 TB6612 电机驱动逻辑与文档 |
| 作者 | Hainzcn |
| 日期 | 2026-07-27 |
| 摘要 | 按 hardware → middle → app 分层接入厂家八路循迹模块（3 位地址 + 1 位 OUT）；非阻塞扫描状态机；串口 `[gs]` 调试输出；修复 microlib 下心跳序号乱码；传感器联调阶段关闭电机 Demo |

## 1. 任务背景

在电机、编码器、心跳串口已联调的基础上（见 `docs/log/2026-07-27-encoder-rpm-motor-pwm-debug.md`），完成循迹传感器硬件接入。厂家参考例程位于 `docs/Grayscale_Read`（已随任务归档/引用），核心逻辑为：

1. 通过 AD0~AD2 写入通道号 0~7；
2. 地址切换后等待约 50 µs OUT 稳定；
3. 读取 OUT 数字电平（0/1）。

本工程约束：**主循环严格禁止阻塞延时**（不用 `delay_cycles`、`system_delay_us/ms`、厂家 `delay_us` 等 busy-wait），须与 `motor_app_demo_process()` 一样采用分步状态机。

## 2. 提交变更概览

| 类别 | 文件 | 说明 |
| --- | --- | --- |
| 硬件层 | `src/hardware/grayscale_hw.c/h` | A15~A18 GPIO；通道选择；OUT 读取 |
| 中间层 | `src/middle/grayscale.c/h` | 非阻塞扫描状态机；8 路缓存 |
| 应用层 | `src/app/grayscale_app.c/h` | 500 ms 串口 `[gs]` 调试输出 |
| 主程序 | `src/main.c` | 接入 `grayscale_app_init/process` |
| 心跳修复 | `src/hardware/heartbeat_hw.c/h`、`src/middle/heartbeat.c` | ISR 维护序号；修正 printf 格式 |
| 联调配置 | `src/app/motor_app.c` | `MOTOR_APP_DEMO_ENABLE` 改为 `0` |
| 文档 | `docs/pin/pin.md` | 新增八路循迹模块接线与协议说明 |

> 注：本提交未包含 `keil/.eide/eide.yml`；工程虚拟文件夹中灰度相关源文件的注册需在 EIDE 中手动确认或于后续提交补齐。

## 3. 引脚分配

厂家例程使用 PA14~PA17；本工程 **A14 已作状态灯**，地址线整体偏移：

| 模块信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| AD0 | A15 (PA15) | 输出推挽 | 地址 bit0 |
| AD1 | A16 (PA16) | 输出推挽 | 地址 bit1 |
| AD2 | A17 (PA17) | 输出推挽 | 地址 bit2 |
| OUT | A18 (PA18) | 输入上拉 | 数字输出；上拉兼容开漏型 |
| VCC | 5V | — | 与厂家例程一致 |
| GND | GND | — | 共地 |

与现有外设无冲突（已占用：A0/A1 PWM、A10/A11 UART、A14 LED、B2~B5 方向、B7/B9/B10/B11 编码器）。

## 4. 软件架构

```
src/
├── hardware/grayscale_hw   ← GPIO init / select_channel / read_out（无延时）
├── middle/grayscale        ← 非阻塞扫描状态机 + values[8] 缓存
└── app/grayscale_app       ← grayscale_process 驱动 + 串口调试
```

### 4.1 中间层状态机（`grayscale_process`）

| 状态 | 行为 |
| --- | --- |
| `SELECT` | 写 AD0~AD2；记录 `SysTick->VAL`（只读） |
| `WAIT_SETTLE` | 轮询 VAL 差值 ≥ 50 µs；未到则 **return** |
| `READ` | 读 OUT 写入 `values[channel]`；channel++；8 路完成后 `scan_ready=1` |

对外 API：

- `grayscale_process()` — 主循环每轮调用，立即返回
- `grayscale_get_values()` — 最近一轮 8 路快照
- `grayscale_is_scan_ready()` — 是否至少完成过一轮扫描

全轮扫描物理时间约 8×50 µs ≈ 400 µs，分散在多次主循环迭代中，不阻塞 heartbeat / motor。

### 4.2 厂家例程缺陷与本项目修正

| 问题 | 厂家例程 | 本项目 |
| --- | --- | --- |
| GPIO 初始化 | `Grayscale_Sensor_Init()` 为空 | `grayscale_hw_init()` 显式 `gpio_init` |
| 延时 | 阻塞 `delay_us(50)` × 8 | SysTick VAL 只读轮询 + 状态机 |
| 同步 API | `Grayscale_Sensor_Read_All()` 一次阻塞扫完 | `grayscale_process()` 分步推进 |
| 返回类型 | `uint16_t` 存 0/1 | `uint8_t` |
| 头文件 | 耦合 `usart.h` | 驱动层仅依赖 GPIO |
| 引脚 | AD0=PA14 与 LED 冲突 | AD0~AD2/OUT → A15~A18 |

## 5. 串口协议

| 报文 | 周期 | 格式 | 示例 |
| --- | --- | --- | --- |
| 上电 | 一次 | `BOOT OK\r\n` | `BOOT OK` |
| 心跳 | 1 s | `[hb] <序号>,<左RPM>,<右RPM>\r\n` | `[hb] 3,0,0` |
| 灰度 | 500 ms | `[gs],<序号>,v0,…,v7\r\n` | `[gs],2,0,0,1,1,0,0,0,0` |

`v0`~`v7` 对应 X1~X8；厂家说明：灯亮/检测到为 1。

主循环调用顺序：

```c
while (1)
{
    heartbeat_app_process();
    motor_app_demo_process();
    grayscale_app_process();
}
```

## 6. 心跳计数异常与修复

### 6.1 现象

接入灰度模块后，上电报文类似：

```
BOOT OK
[gs],1,1,1,1,1,1,1,1,1
[gs],2,1,1,1,1,1,1,1,1
[hb] 538968613,0,0
```

`[hb]` 序号固定为 `538968613`（`0x20202025`，ASCII `"%   "`），不递增；`[gs]` 序号正常。

### 6.2 根因

Keil microlib 的 `snprintf` 在 **`[hb] %lu,%ld,%ld`** 格式下 varargs 对齐异常，第一个 `%lu` 误读格式串附近内存。`[gs]` 报文使用 `%lu` 后跟 `%u` 未触发同样问题。

原先在 `heartbeat.c` 用局部变量 `heartbeat_tick_count++` 再 `%lu` 打印，不可靠。

### 6.3 修复（同提交内）

1. 在 `heartbeat_hw.c` 的 SysTick ISR 中维护 `heartbeat_hw_sequence`，与 `pending_ticks` 同步递增。
2. `heartbeat_send()` 改为读取 `heartbeat_hw_get_sequence()`，格式改为 **`[hb] %u,%d,%d`**。
3. `[gs]` 序号统一为 `%u`，避免 `%lu`。

修复后预期：`[hb] 1,0,0` → `[hb] 2,0,0` → …

## 7. 板端联调记录

| 现象 | 分析 |
| --- | --- |
| `[gs]` 八路全为 `1` | OUT 上拉默认高；可能模块未接/供电异常，或探头均对白底（高有效） |
| `[hb]` 序号乱码 | 已由 §6 修复；需重新编译烧录验证 |
| `[gs]` 每 500 ms、`[hb]` 每 1 s | 说明 `heartbeat_hw_ms` / SysTick 节拍正常 |
| 电机 Demo 已关 | `MOTOR_APP_DEMO_ENABLE=0`，避免转动干扰传感器读数 |

## 8. 验证状态

| 项目 | 状态 |
| --- | --- |
| 分层代码与 `pin.md` 文档 | 已提交 `f40006d` |
| 非阻塞扫描（无 busy-wait） | 已实现 |
| 串口 `[gs]` 调试输出 | 已实现，板端可收到 |
| 心跳序号修复 | 已合入同提交，待复测 |
| 8 路 0/1 随黑线变化 | 待硬件接线与模块供电确认 |
| EIDE 源文件注册 | 提交外需确认 `eide.yml` |

## 9. 设计约束（更新）

1. **循迹 GPIO**：A15~A18 由逐飞 `gpio_init` 运行时初始化；**不修改** `M0G3519.syscfg`。
2. **禁止阻塞延时**：传感器路径及主循环不得使用 `delay_cycles`、`system_delay_*`、厂家 `delay_us`。
3. **SysTick 共用**：心跳 1 ms ISR 与灰度 VAL 轮询共存；灰度层只读 `SysTick->VAL/LOAD`，不改 CTRL/LOAD。
4. **printf**：嵌入式串口格式化优先 `%u`/`%d`，避免 microlib 下 `%lu`/`%ld` 混用。
5. **心跳序号**：以 `heartbeat_hw_sequence`（ISR 递增）为唯一来源，不在 middle 层单独维护计数。

## 10. 后续计划

- [x] 复测 `[hb]` 序号递增与 LED 1 Hz 闪烁正常。
- [x] 确认模块 5V/GND、AD0~AD2/OUT 接线后，8 路随黑线/白纸切换 0/1。
- [ ] 实现加权偏差（line position）与循迹控制（`motor_set_speed` 差速）。
- [ ] 同步 `docs/api/README.md` 中 `[hb]`/`[gs]` 协议说明。

## 11. 相关文档

- 引脚与接线：`docs/pin/pin.md`（「八路循迹模块」章节）
- 厂家参考例程：`docs/Grayscale_Read/`（BSP `grayscale_sensor.c/h`）
- 上一阶段日志：`docs/log/2026-07-27-encoder-rpm-motor-pwm-debug.md`（父提交 `b8ba0f6` 之前）
