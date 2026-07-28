# MSPM0G3519 循迹小车 — 应用 API 说明

本文档描述截至提交 `a187d84` 已实现的 **motor**、**encoder**、**heartbeat**、**grayscale**、**imu**、**oled** 模块对外接口。项目采用 **hardware → middle → app** 三层架构；上层模块应优先调用 app 或 middle 层接口，避免跨层直接操作 hardware。

引脚与接线详见 [docs/pin/pin.md](../pin/pin.md)。任务变更记录见 [docs/log/](../log/)。

## 架构概览

```
main.c
  ├── clock_init()              [逐飞库 / SysConfig]
  ├── heartbeat_app_*()         [app → middle → hardware]
  ├── motor_app_*()             [app → middle → hardware]
  ├── grayscale_app_*()         [app → middle → hardware]
  ├── imu_app_*()               [app → middle → hardware]
  └── oled_app_*()              [app → middle → hardware]
```

| 层级 | 电机 | 编码器 | 心跳 | 循迹 | IMU | OLED |
| --- | --- | --- | --- | --- | --- | --- |
| app | `motor_app.h` | — | `heartbeat_app.h` | `grayscale_app.h` | `imu_app.h` | `oled_app.h` |
| middle | `motor.h` | `encoder.h` | `heartbeat.h` | `grayscale.h` | `imu.h` | `oled.h` |
| hardware | `motor_hw.h` | `encoder_hw.h` | `heartbeat_hw.h` | `grayscale_hw.h` | `imu_hw.h` | `oled_hw.h` |

`motor_init()` 内部会调用 `encoder_init()`；编码器 RPM 在心跳周期内更新。OLED 仪表盘读取 IMU 航向、编码器 RPM 与灰度缓存。

---

## 电机模块（motor）

### 应用层 — `src/app/motor_app.h`

#### `void motor_app_init(void)`

上电初始化电机子系统。内部调用 `motor_init()`（含编码器初始化），完成后左右轮保持停止。

- **调用时机**：`main()` 中，建议在 `heartbeat_app_init()` 之后调用

#### `void motor_app_demo_process(void)`

非阻塞电机转动测试，在 `main()` 主循环中反复调用。

配置宏位于 `src/app/motor_app.c` 顶部：

| 宏 | 当前默认 | 说明 |
| --- | --- | --- |
| `MOTOR_APP_DEMO_ENABLE` | `0` | `1`=运行 Demo，`0`=关闭 |
| `MOTOR_APP_DEMO_HOLD_FORWARD` | `1` | `1`=持续正转满速；`0`=正反转交替 |
| `MOTOR_APP_DEMO_DUTY` | `10000` | PWM 占空比 |
| `MOTOR_APP_DEMO_STEP_MS` | `2000` | 正/反转各持续 ms（`HOLD_FORWARD=0` 时） |
| `MOTOR_APP_DEMO_LOOP` | `1` | 是否循环正反转 |

---

### 中间层 — `src/middle/motor.h`

#### 常量

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_SPEED_MAX` | 10000 | 归一化速度满量程 |

#### `void motor_init(void)`

初始化 motor_hw、encoder，并停止所有电机。

#### `void motor_set_speed(int32 left_speed, int32 right_speed)`

设置左右轮速度（开环直通映射）。

| 参数 | 类型 | 范围 | 说明 |
| --- | --- | --- | --- |
| `left_speed` | `int32` | [-10000, 10000] | 左轮；正为正向，负为反向，0 为滑行停止 |
| `right_speed` | `int32` | [-10000, 10000] | 右轮；同上 |

#### `void motor_brake(void)` / `void motor_stop(void)`

左右轮短路刹车 / 滑行停止（占空比 0）。

---

### 硬件层 — `src/hardware/motor_hw.h`

TB6612 **标准双 PWM + GPIO 方向** 接法：

| 信号 | 引脚 | TIM_A0 |
| --- | --- | --- |
| 左 PWMA | A0 | CH0 |
| 右 PWMB | A1 | CH1 |
| 左 AIN1 / AIN2 | B2 / B3 | — |
| 右 BIN1 / BIN2 | B4 / B5 | — |

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_HW_PWM_FREQ_HZ` | 17000 | PWM 频率（Hz） |
| `MOTOR_HW_DUTY_MAX` | 10000 | 占空比满量程 |

主要接口：`motor_hw_init()`、`motor_hw_set_duty()`、`motor_hw_brake()`、`motor_hw_stop_all()`。

---

## 编码器模块（encoder）

### 中间层 — `src/middle/encoder.h`

GM37-520 + 11 线编码器，减速比 30:1，QEI 4 倍频，输出轴每转 **1320** counts。

#### 常量

| 名称 | 值 |
| --- | --- |
| `ENCODER_COUNTS_PER_WHEEL_REV` | 1320 |
| `ENCODER_RAW_COUNT_MOD` | 65536 |

#### 初始化与计数

| 函数 | 说明 |
| --- | --- |
| `encoder_init()` | 初始化 QEI；由 `motor_init()` 调用 |
| `encoder_get_left_raw_count()` / `encoder_get_right_raw_count()` | 16 位原始计数 0~65535 |
| `encoder_get_left_total_count()` / `encoder_get_right_total_count()` | 环形差分累加后的 `int32` 总里程 |
| `encoder_clear_left_count()` / `encoder_clear_right_count()` / `encoder_clear_all_count()` | 清零 |

#### 转速

| 函数 | 说明 |
| --- | --- |
| `encoder_update_speed(uint32 period_ms)` | 在已知时间间隔内差分计数并换算 RPM；由心跳模块调用 |
| `encoder_get_left_rpm()` / `encoder_get_right_rpm()` | 最近一次更新后的输出轴 RPM |

RPM 换算：`rpm = Δcount × 60000 / (1320 × period_ms)`

### 硬件层 — `src/hardware/encoder_hw.h`

| 侧别 | 定时器 | A 相 | B 相 |
| --- | --- | --- | --- |
| 左轮 | TIMG8 | B10 | B11 |
| 右轮 | TIMG9 | B7 | B9 |

主要接口：`encoder_hw_init()`、`encoder_hw_get_raw_count()`、`encoder_hw_clear_count()`。

---

## 心跳模块（heartbeat）

### 应用层 — `src/app/heartbeat_app.h`

#### `void heartbeat_app_init(void)`

启动状态灯与 UART0 心跳。上电后立即发送 `BOOT OK\r\n`。

- **前置条件**：`main()` 中已调用 `clock_init(SYSTEM_CLOCK_80M)`

#### `void heartbeat_app_process(void)`

处理待发送的心跳周期；在 `main()` 的 `while(1)` 中反复调用，**不可在中断中调用**。

---

### 中间层 — `src/middle/heartbeat.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `HEARTBEAT_PERIOD_MS` | 1000 | 心跳周期 1 Hz（LED 翻转 + 串口报文 + RPM 更新） |

| 函数 | 说明 |
| --- | --- |
| `heartbeat_init()` | 初始化 hardware 层并发送 `BOOT OK\r\n` |
| `heartbeat_process()` | 检查周期标志；有则翻转 LED、更新 RPM、发送 `[hb]` |
| `heartbeat_get_ms()` | 自 SysTick 启动以来的毫秒计数；供非阻塞定时（如 Demo、灰度/OLED/IMU 调试） |

---

### 硬件层 — `src/hardware/heartbeat_hw.h`

| 函数 | 说明 |
| --- | --- |
| `heartbeat_hw_init(tick_period_ms)` | PA14 LED + SysTick 1 ms；**不初始化 UART** |
| `heartbeat_hw_led_toggle()` | 翻转 PA14 |
| `heartbeat_hw_uart_send_string(str)` | UART0 阻塞发送 |
| `heartbeat_hw_take_tick()` | 原子取走一个待处理周期标志 |
| `heartbeat_hw_get_ms()` | 毫秒计数 |
| `heartbeat_hw_get_sequence()` | 心跳周期序号（SysTick ISR 中递增） |

---

## 循迹模块（grayscale）

八路数字灰度模块：**3 位地址 AD0~AD2 + 1 位 OUT**，非阻塞扫描，禁止主循环 busy-wait 延时。

引脚：AD0=A15，AD1=A16，AD2=A17，OUT=A18（相对厂家例程 PA14~PA17 偏移，避开 A14 状态灯）。

### 应用层 — `src/app/grayscale_app.h`

| 函数 | 说明 |
| --- | --- |
| `grayscale_app_init()` | 调用 `grayscale_init()` |
| `grayscale_app_process()` | 推进扫描状态机；每 500 ms 通过 UART0 发送 `[gs]` 调试报文 |

### 中间层 — `src/middle/grayscale.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `GRAYSCALE_CHANNELS` | 8 | 探头数量 X1~X8 |

| 函数 | 说明 |
| --- | --- |
| `grayscale_init()` | 初始化 GPIO 与扫描状态机 |
| `grayscale_process()` | 主循环每轮调用，立即返回；分步完成 8 路扫描 |
| `grayscale_get_values()` | 指向长度 8 的 `uint8` 数组，元素为 0 或 1 |
| `grayscale_is_scan_ready()` | 是否至少完成过一轮扫描 |

地址切换后约 50 µs 稳定时间通过 **只读 SysTick VAL 轮询** 实现（见 `grayscale.c`）。

### 硬件层 — `src/hardware/grayscale_hw.h`

| 函数 | 说明 |
| --- | --- |
| `grayscale_hw_init()` | AD0~AD2 推挽输出，OUT 上拉输入 |
| `grayscale_hw_select_channel(ch)` | 写入通道号 0~7 |
| `grayscale_hw_read_out()` | 读取 OUT 电平 0/1 |

---

## IMU 模块（imu）

单轴 IMU 模块通过 **UART1** 主动推送 Z 轴角速度与航向角；MCU 侧 **5 字节**读帧，主循环非阻塞解析。详见 `docs/数据手册(串口通信).pdf` 与 [docs/log/2026-07-27-imu-uart-module.md](../log/2026-07-27-imu-uart-module.md)。

引脚：MCU TX=A8 → 模块 RX；模块 TX → MCU RX=A9；115200-8-N-1。

### 应用层 — `src/app/imu_app.h`

| 函数 | 说明 |
| --- | --- |
| `imu_app_init()` | 调用 `imu_init()`；启动 500 ms boot 等待状态机 |
| `imu_app_process()` | 推进 boot/Yaw 归零状态机；调用 `imu_process()`；输出 `[imu]` 调试 |

配置宏（`src/app/imu_app.c`）：

| 宏 | 默认 | 说明 |
| --- | --- | --- |
| `IMU_APP_YAW_ZERO_ON_BOOT` | `0` | `1`=上电非阻塞执行 Yaw 归零（寄存器 0x15） |
| `IMU_APP_BOOT_DELAY_MS` | 500 | boot 后开启 UART1 RX 中断的等待时间 |
| `IMU_APP_DEBUG_PERIOD_MS` | 1000 | 就绪后 `[imu]` 输出周期 |
| `IMU_APP_WAIT_DEBUG_PERIOD_MS` | 2000 | 未就绪时 `wait` 报文周期 |

就绪条件：同时收到 TYPE `0xAA`（Wz）与 `0xBB`（Yaw）帧。

### 中间层 — `src/middle/imu.h`

#### 协议常量

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `IMU_FRAME_HEADER` | 0x5A | 读帧帧头 |
| `IMU_FRAME_SIZE` | 5 | 读帧长度 |
| `IMU_TYPE_GYRO` | 0xAA | Z 轴角速度 |
| `IMU_TYPE_ANGLE` | 0xBB | 航向角 Yaw |
| `IMU_FLAG_GYRO` / `IMU_FLAG_ANGLE` | 0x01 / 0x02 | 数据就绪标志 |

#### 数据结构

```c
typedef struct { float wz;  } imu_gyro_t;
typedef struct { float yaw; } imu_angle_t;
typedef struct {
    imu_gyro_t  gyro;
    imu_angle_t angle;
    uint8       flags;
} imu_snapshot_t;
```

| 函数 | 说明 |
| --- | --- |
| `imu_init()` | 初始化 hardware 层与解析 FSM |
| `imu_process()` | 主循环调用；从 RX FIFO 消费字节并更新缓存 |
| `imu_get_gyro()` / `imu_get_angle()` | 最近解码结果指针 |
| `imu_get_update_flags()` | 已收到帧类型位掩码 |
| `imu_get_snapshot(snapshot)` | 拷贝当前快照 |
| `imu_is_type_ready(flag)` | 指定类型是否至少收到一帧 |

物理量换算：16 位有符号 raw → Wz 量程 ±400°/s，Yaw ±180°。

### 硬件层 — `src/hardware/imu_hw.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `IMU_HW_RX_FIFO_SIZE` | 512 | RX 环形缓冲 |
| `IMU_HW_TX_TIMEOUT_CYCLES` | 8000000 | TX 超时（约 100 ms @ 80 MHz） |

| 函数 | 说明 |
| --- | --- |
| `imu_hw_init()` | UART1 由 SysConfig 初始化；配置 RX FIFO |
| `imu_hw_rx_enable()` | 开启 UART1 RX 中断（boot 等待后调用） |
| `imu_hw_read_byte(byte)` | 从 FIFO 取一字节；无数据返回 0 |
| `imu_hw_write_frame(frame, len)` | 发送写帧（带 TX 超时） |
| `imu_hw_write_reg(addr, value)` | 写寄存器命令 `55 AA ADDR DATAL DATAH` |

---

## OLED 模块（oled）

GME12864-49（128×64，SSD1306/兼容），**硬件 I2C0** @ 400 kHz。详见 [docs/log/2026-07-28-oled-display-module.md](../log/2026-07-28-oled-display-module.md)。

引脚：SCL=B0，SDA=B1；VCC=3.3V；I2C 地址默认 0x3C。

### 应用层 — `src/app/oled_app.h`

| 函数 | 说明 |
| --- | --- |
| `oled_app_init()` | 初始化 OLED、绘制静态标签、首屏刷新 |
| `oled_app_process()` | 增量更新 Yaw/RPM/循迹条；仅数据变化时推送 I2C |

仪表盘布局：

| 页 | 内容 | 刷新 |
| --- | --- | --- |
| 0 | 标题 `MSPM0G3519` | init 一次 |
| 1 | `Yaw:` + 整数航向 | 100 ms，变化时推送 |
| 3 | `L:` / `R:` + 左右 RPM | 100 ms，变化时推送 |
| 7 | 八路循迹条（每路 16 px） | 50 ms，变化时推送 |

未检测到 I2C ACK 时 `oled_app_process()` 立即返回，不影响其他模块。

### 中间层 — `src/middle/oled.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `OLED_FONT_6X8` / `OLED_FONT_8X16` | 0 / 1 | 字库尺寸 |
| 分辨率 | 128×64 | 1 KB 帧缓冲 |

| 函数 | 说明 |
| --- | --- |
| `oled_init()` | 初始化并清屏刷新 |
| `oled_is_ready()` | I2C 探测是否成功 |
| `oled_clear()` | 清帧缓冲（不推送） |
| `oled_set_pixel(x, y, on)` | 设单像素 |
| `oled_show_char()` / `oled_show_string()` | 6×8 或 8×16 文本 |
| `oled_show_uint()` / `oled_show_int()` | 数值显示 |
| `oled_clear_area(x, y, w, h)` | 矩形清缓冲 |
| `oled_clear_page(page)` | 清整页 |
| `oled_clear_page_segment(page, x, w)` | 清页内水平段 |
| `oled_fill_page_bar(page, values, count, block_width)` | 循迹条直写页缓冲 |
| `oled_refresh()` | 整屏推送 I2C |
| `oled_refresh_pages(page_start, page_end)` | 按页范围推送 |

### 硬件层 — `src/hardware/oled_hw.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `OLED_HW_WIDTH` / `OLED_HW_HEIGHT` | 128 / 64 | 分辨率 |
| `OLED_HW_I2C_ADDR` | 0x3C | 7 位地址（SA0 接高改 0x3D） |

| 函数 | 说明 |
| --- | --- |
| `oled_hw_init()` | I2C0 + SSD1306 初始化序列 |
| `oled_hw_is_ready()` | 初始化是否成功 |
| `oled_hw_write_cmd(cmd)` | 写命令字节 |
| `oled_hw_write_data(data, len)` | 写显存数据 |

---

## 系统入口 — `src/main.c`

```c
#include "grayscale_app.h"
#include "heartbeat_app.h"
#include "imu_app.h"
#include "motor_app.h"
#include "oled_app.h"
#include "zf_common_clock.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    motor_app_init();
    grayscale_app_init();
    imu_app_init();
    oled_app_init();

    while (1)
    {
        heartbeat_app_process();
        motor_app_demo_process();
        grayscale_app_process();
        imu_app_process();
        oled_app_process();
    }
}
```

### `clock_init(uint32 clock)` — `zf_common_clock.h`

内部完成外设上电、`SYSCFG_DL_init()`（含 UART0 PA10/PA11、UART1 PA8/PA9、I2C0 PB0/PB1）、`interrupt_init()`。

---

## 串口协议

**参数**：UART0，115200，8-N-1，无流控。TX=PA10，RX=PA11。

| 时机 | 报文格式 | 示例 |
| --- | --- | --- |
| 上电 | `BOOT OK\r\n` | `BOOT OK` |
| 每 1 s | `[hb] <序号>,<左RPM>,<右RPM>\r\n` | `[hb] 3,120,-118` |
| 每 500 ms | `[gs],<序号>,v0,v1,…,v7\r\n` | `[gs] 2,0,0,1,1,0,0,0,0` |
| 每 1 s（IMU 就绪） | `[imu] <序号>,<yaw>,<wz>\r\n` | `[imu] 3,12.34,-0.56` |
| IMU 未就绪 | `[imu] 0,wait,flags=0x??\r\n` | `[imu] 0,wait,flags=0x01` |

- `[hb]` 序号由 `heartbeat_hw_get_sequence()` 提供，在 SysTick ISR 中递增。
- `[gs]` 的 `v0`~`v7` 对应 X1~X8；厂家说明：灯亮/检测到为 1。
- `[imu]` 的 yaw、wz 为浮点（° 与 °/s，两位小数）；就绪需 `IMU_FLAG_ANGLE | IMU_FLAG_GYRO`。
- 串口格式化使用 `%u`/`%d`/`%.2f`，避免 microlib 下 `%lu`/`%ld` 混用导致乱码。

IMU 模块通信走 **UART1**（A8/A9），与 UART0 调试口独立。

---

## 调用示例

### 循迹应用中设置电机速度

```c
#include "motor.h"

void line_follow_update(int32 err)
{
    int32 base = 3000;
    int32 turn = err * 50;

    motor_set_speed(base - turn, base + turn);
}
```

### 读取 IMU 航向（middle 层）

```c
#include "imu.h"

void control_update(void)
{
    imu_process();

    if (imu_is_type_ready(IMU_FLAG_ANGLE))
    {
        const imu_angle_t *angle = imu_get_angle();
        /* angle->yaw 单位：度 */
    }
}
```

### 在 OLED 上显示自定义文本

```c
#include "oled.h"

void show_status(const char *msg)
{
    if (!oled_is_ready())
    {
        return;
    }

    oled_clear_page(2);
    oled_show_string(0, 2, msg, OLED_FONT_6X8);
    oled_refresh_pages(2, 2);
}
```

### 读取循迹传感器（middle 层）

```c
#include "grayscale.h"

void app_loop(void)
{
    const uint8 *values;

    grayscale_process();

    if (grayscale_is_scan_ready())
    {
        values = grayscale_get_values();
        /* values[0]..values[7] 为 0 或 1 */
    }
}
```

### 非阻塞定时（基于心跳毫秒）

```c
#include "heartbeat.h"

static uint32 last_ms;

void periodic_task(void)
{
    uint32 now = heartbeat_get_ms();
    if ((now - last_ms) >= 100)
    {
        last_ms = now;
        /* 每 100 ms 执行一次 */
    }
}
```

### 调整心跳 / 灰度 / OLED / IMU 周期

- 心跳：修改 `src/middle/heartbeat.h` 中 `HEARTBEAT_PERIOD_MS`
- 灰度串口：`src/app/grayscale_app.c` 中 `GRAYSCALE_APP_DEBUG_PERIOD_MS`
- IMU 串口：`src/app/imu_app.c` 中 `IMU_APP_DEBUG_PERIOD_MS`
- OLED 数值/循迹：`src/app/oled_app.c` 中 `OLED_APP_TEXT_PERIOD_MS` / `OLED_APP_GS_PERIOD_MS`

---

## 依赖与约束

1. 电机 PWM 占用 **TIM_A0**（A0/A1）；编码器 QEI 占用 **TIMG8/TIMG9**；心跳与灰度稳定计时共用 **SysTick**。
2. **UART0**、**UART1**、**I2C0** 必须由 **SysConfig** 初始化；不要对 UART0 调用逐飞 `uart_init()`。
3. **禁止阻塞延时**：主循环及传感器/IMU 路径不得使用 `delay_cycles`、`system_delay_us/ms` 等 busy-wait；IMU init 与 Yaw 归零均为状态机。
4. 灰度 GPIO（A15~A18）由逐飞 `gpio_init` 运行时初始化，无需修改 `M0G3519.syscfg`。
5. **OLED I2C 刷新**须在主循环调用；禁止在 ISR 中调用 `oled_refresh*`。
6. IMU UART1 TX 带超时；RX 在 boot 500 ms 后才开启，避免上电阻塞。
7. 传感器联调阶段建议 `MOTOR_APP_DEMO_ENABLE = 0`，避免电机转动干扰读数。
8. 调试断言与 FIFO（`zf_common_debug`、`zf_common_fifo`）供逐飞库内部使用；应用层无需直接调用。

---

## 后续扩展（尚未实现）

- `motor_set_target_rpm(left_rpm, right_rpm)` — 编码器闭环 PID
- 加权偏差计算与循迹控制算法
- IMU 输出速率调整（unlock → RRATE → SAVE）
- OLED 浮点 Yaw 显示与控制状态行
