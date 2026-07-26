# MSPM0G3519 循迹小车 — 应用 API 说明

本文档描述提交 `a74c950` 中已实现的 **motor** 与 **heartbeat** 模块对外接口。项目采用 **hardware → middle → app** 三层架构；上层模块应优先调用 app 或 middle 层接口，避免跨层直接操作 hardware。

## 架构概览

```
main.c
  ├── clock_init()              [逐飞库 / SysConfig]
  ├── heartbeat_app_*()         [app → middle → hardware]
  └── motor_app_*()             [app → middle → hardware]
```

| 层级 | 电机模块 | 心跳模块 |
| --- | --- | --- |
| app | `motor_app.h` | `heartbeat_app.h` |
| middle | `motor.h` | `heartbeat.h` |
| hardware | `motor_hw.h` | `heartbeat_hw.h` |

引脚与接线详见 [docs/pin/pin.md](../pin/pin.md)。

---

## 电机模块（motor）

### 应用层 — `src/app/motor_app.h`

#### `void motor_app_init(void)`

上电初始化电机子系统。内部调用 `motor_init()`，完成后左右轮保持停止。

- **参数**：无
- **返回值**：无
- **调用时机**：`main()` 中，建议在 `heartbeat_app_init()` 之后调用
- **副作用**：初始化 TIM_A0 四路 PWM，占空比为 0

#### `void motor_app_demo(void)`

电机接线自检：正转 0.5 s → 反转 0.5 s → 停止。

- **参数**：无
- **返回值**：无
- **说明**：默认不在 `main` 中调用；联调时在断点或临时代码中手动触发
- **速度**：半速（`MOTOR_SPEED_MAX / 2`）

---

### 中间层 — `src/middle/motor.h`

#### 常量

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_SPEED_MAX` | 10000 | 归一化速度满量程，与 hardware 层占空比对齐 |

#### `void motor_init(void)`

初始化 hardware 层并停止所有电机。

#### `void motor_set_speed(int32 left_speed, int32 right_speed)`

设置左右轮速度（开环直通映射）。

| 参数 | 类型 | 范围 | 说明 |
| --- | --- | --- | --- |
| `left_speed` | `int32` | [-10000, 10000] | 左轮；正为正向，负为反向，0 为滑行停止 |
| `right_speed` | `int32` | [-10000, 10000] | 右轮；同上 |

超出范围的值会被截断。

#### `void motor_brake(void)`

左右轮同时短路刹车（TB6612 双臂满占空比）。

#### `void motor_stop(void)`

左右轮滑行停止（占空比置 0）。

**扩展示例（后续闭环）**：

```c
// 预留：接入编码器后可新增
// void motor_set_target_rpm(int32 left_rpm, int32 right_rpm);
// 内部 PID 计算后仍调用 motor_set_speed() 下发
```

---

### 硬件层 — `src/hardware/motor_hw.h`

一般仅供 middle 层调用。

#### 常量

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_HW_PWM_FREQ_HZ` | 17000 | PWM 频率（Hz） |
| `MOTOR_HW_DUTY_MAX` | 10000 | 占空比满量程 |

#### 枚举 `motor_hw_index_enum`

| 值 | 说明 |
| --- | --- |
| `MOTOR_HW_LEFT` | 左电机 |
| `MOTOR_HW_RIGHT` | 右电机 |

#### `void motor_hw_init(void)`

初始化 TIM_A0 四路 PWM（A0/A1/B12/B13），初始占空比 0。

#### `void motor_hw_set_duty(motor_hw_index_enum motor, int32 duty)`

单路占空比控制。

| 参数 | 说明 |
| --- | --- |
| `motor` | `MOTOR_HW_LEFT` 或 `MOTOR_HW_RIGHT` |
| `duty` | [-10000, 10000]；符号决定转向，0 为滑行 |

#### `void motor_hw_brake(motor_hw_index_enum motor)`

单路短路刹车。

#### `void motor_hw_stop_all(void)`

停止所有电机（占空比 0）。

---

## 心跳模块（heartbeat）

### 应用层 — `src/app/heartbeat_app.h`

#### `void heartbeat_app_init(void)`

启动状态灯与 UART0 心跳。上电后立即发送 `BOOT OK\r\n`。

- **前置条件**：`main()` 中已调用 `clock_init(SYSTEM_CLOCK_80M)`（含 UART0 SysConfig 初始化）

#### `void heartbeat_app_process(void)`

处理待发送的心跳周期。

- **调用方式**：在 `main()` 的 `while(1)` 中反复调用
- **不可在中断中调用**

---

### 中间层 — `src/middle/heartbeat.h`

#### 常量

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `HEARTBEAT_PERIOD_MS` | 500 | 心跳周期（ms），LED 翻转 + 串口报文 |

#### `void heartbeat_init(void)`

初始化 hardware 层并发送 `BOOT OK\r\n`。

#### `void heartbeat_process(void)`

检查 SysTick 累积的周期标志；若有待处理周期，则翻转 LED 并发送：

```text
HEARTBEAT,<计数值>\r\n
```

计数值从 1 递增。

---

### 硬件层 — `src/hardware/heartbeat_hw.h`

#### `void heartbeat_hw_init(uint32 tick_period_ms)`

- 初始化 PA14 LED（推挽输出，默认熄灭）
- 配置 SysTick 1 ms 中断
- **不初始化 UART**（由 SysConfig 在 `clock_init()` 中完成）

| 参数 | 说明 |
| --- | --- |
| `tick_period_ms` | 心跳周期，通常传入 `HEARTBEAT_PERIOD_MS`（500） |

#### `void heartbeat_hw_led_toggle(void)`

翻转 PA14 电平。

#### `void heartbeat_hw_uart_send_string(const char *str)`

通过 UART0 阻塞发送以 `\0` 结尾的字符串。

| 参数 | 说明 |
| --- | --- |
| `str` | 待发送字符串；`NULL` 时不发送 |

底层使用 `DL_UART_Main_transmitDataBlocking(UART_0_INST, ...)`。

#### `uint8 heartbeat_hw_take_tick(void)`

原子取走一个待处理周期标志。

| 返回值 | 说明 |
| --- | --- |
| 1 | 本次有待处理周期 |
| 0 | 无待处理周期 |

---

## 系统入口 — `src/main.c`

```c
#include "heartbeat_app.h"
#include "motor_app.h"
#include "zf_common_clock.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    motor_app_init();

    while (1)
    {
        heartbeat_app_process();
    }
}
```

### `clock_init(uint32 clock)` — `zf_common_clock.h`

| 参数 | 说明 |
| --- | --- |
| `clock` | 系统时钟，使用 `SYSTEM_CLOCK_80M`（80 MHz） |

内部完成外设上电、`SYSCFG_DL_init()`（含 UART0）、中断分组初始化。

---

## 串口协议

| 时机 | 报文格式 | 示例 |
| --- | --- | --- |
| 上电 | `BOOT OK\r\n` | `BOOT OK` |
| 每 500 ms | `HEARTBEAT,<n>\r\n` | `HEARTBEAT,1` |

**串口参数**：UART0，115200，8-N-1，无流控。TX=PA10，RX=PA11。

---

## 调用示例

### 循迹应用中设置电机速度

```c
#include "motor.h"

void line_follow_update(int32 err)
{
    int32 base = 3000;
    int32 turn = err * 50;   // 示例增益

    motor_set_speed(base - turn, base + turn);
}
```

### 临时触发电机自检

```c
#include "motor_app.h"

// 在 main 初始化完成后、进入主循环前调用一次
motor_app_demo();
```

### 调整心跳周期

修改 `src/middle/heartbeat.h` 中的 `HEARTBEAT_PERIOD_MS`，重新编译即可。

---

## 依赖与约束

1. 电机 PWM 占用 **TIM_A0**，心跳节拍使用 **SysTick**，两者互不冲突。
2. UART0 必须由 **SysConfig** 初始化；不要对 UART0 调用逐飞 `uart_init()`。
3. 编码器相关 API（`encoder_quad_init` 等）在逐飞库中可用，但当前工程**未接入**；详见 [docs/pin/pin.md](../pin/pin.md) 占位说明。
4. 调试断言与 FIFO 相关代码已编入工程（`zf_common_debug`、`zf_common_fifo`），供逐飞库内部使用；应用层无需直接调用。
