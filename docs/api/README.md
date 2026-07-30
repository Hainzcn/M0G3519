# MSPM0G3519 循迹小车 — 应用 API 说明

本文档描述截至提交 `8b0a33c` 已实现模块的对外接口：**motor**、**encoder**、**heartbeat**、**grayscale**、**imu**、**oled**，以及 **100 Hz 底盘控制栈**（`control_config`、`control_pid`、`line_control`、`wheel_speed_control`、`motor_app`）。

项目采用 **hardware → middle → app** 三层架构；上层应优先调用 app 或 middle 层接口，避免跨层直接操作 hardware。

- 引脚与接线：[docs/pin/pin.md](../pin/pin.md)
- 控制框架与调参：[docs/循迹两驱PID框架.md](../循迹两驱PID框架.md)
- 任务变更记录：[docs/log/](../log/)

## 架构概览

```
main.c
  ├── clock_init()                  [逐飞库 / SysConfig]
  ├── heartbeat_app_*()             [app → middle → hardware]
  ├── grayscale_app_*()
  ├── motor_app_*()                 [100 Hz 控制调度]
  ├── imu_app_*()
  └── oled_app_*()
```

控制数据流（落地循迹）：

```text
grayscale → line_control（循迹 PID + 曲率前馈）
         → wheel_speed_control（双轮速度 PID + 前馈）
         → motor（极性/交换）→ motor_hw（TB6612）
         ← encoder（100 Hz 测速）
```

| 层级 | 电机/控制 | 编码器 | 心跳 | 循迹 | IMU | OLED |
| --- | --- | --- | --- | --- | --- | --- |
| app | `motor_app.h` | — | `heartbeat_app.h` | `grayscale_app.h` | `imu_app.h` | `oled_app.h` |
| middle | `motor.h`、`line_control.h`、`wheel_speed_control.h`、`control_pid.h` | `encoder.h` | `heartbeat.h` | `grayscale.h` | `imu.h` | `oled.h` |
| config | `control_config.h`（宏，无 .c） | — | — | — | — | — |
| hardware | `motor_hw.h` | `encoder_hw.h` | `heartbeat_hw.h` | `grayscale_hw.h` | `imu_hw.h` | `oled_hw.h` |

`motor_init()` 内部调用 `encoder_init()`。编码器 RPM 由 **`wheel_speed_control_update()`** 以 10 ms 周期更新；1 Hz 心跳 `[hb]` 只读取结果。OLED 读取 IMU 航向、编码器 RPM、灰度缓存与循迹输出。

---

## 底盘控制（motor_app + 控制栈）

100 Hz 双层 PID 框架详见 [docs/循迹两驱PID框架.md](../循迹两驱PID框架.md)。所有增益、限幅与标定宏位于 `src/middle/control_config.h`。

### 应用层 — `src/app/motor_app.h`

#### 运行模式 — `motor_app_mode_enum`

| 值 | 名称 | 说明 |
| --- | --- | --- |
| 0 | `MOTOR_APP_MODE_DISABLED` | 停止；轮速环输出 0 |
| 1 | `MOTOR_APP_MODE_SPEED_TEST` | 架空轮速标定 |
| 2 | `MOTOR_APP_MODE_RIGHT_CIRCLE_DEMO` | 顺时针 1 m 圆 demo |
| 3 | `MOTOR_APP_MODE_LINE_FOLLOW` | 落地循迹 |

#### 接口

| 函数 | 说明 |
| --- | --- |
| `motor_app_init()` | 初始化 motor、line_control、wheel_speed_control；按 `control_config.h` 自动启动模式 |
| `motor_app_process()` | **主循环每轮调用**；10 ms 调度控制环；活动模式下每 250 ms 输出 `[ctl]` |
| `motor_app_stop()` | 停止电机并复位控制状态 |
| `motor_app_set_line_follow_enabled(enabled)` | `1`=进入循迹；`0`=同 stop |
| `motor_app_set_base_rpm(base_rpm)` | 设置循迹基准 RPM |
| `motor_app_set_speed_test(left_rpm, right_rpm)` | 进入轮速测试模式 |
| `motor_app_set_right_circle_demo(center_rpm)` | 进入顺时针圆 demo（按轮距/圆直径算差速） |
| `motor_app_get_mode()` | 返回当前模式 |

#### 自动启动宏（`control_config.h`）

| 宏 | 默认 | 说明 |
| --- | --- | --- |
| `MOTOR_APP_AUTO_START_LINE_FOLLOW` | `1` | 上电自动循迹 |
| `MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO` | `0` | 上电自动圆 demo；与上项互斥 |

#### 安全策略

- 灰度 `scan_sequence` 超过 **30 ms** 不更新 → 复位循迹并停车。
- 丢线超过 **200 ms**（`LINE_LOST_HOLD_MS`）→ 清空 PID 并停车。
- 主循环卡顿 **>50 ms** → 跳过一次控制并复位状态。

---

### 循迹外环 — `src/middle/line_control.h`

#### 输出结构 — `line_control_output_t`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `left_rpm` / `right_rpm` | `float` | 左右目标 RPM（含斜率限制） |
| `error` | `float` | 滤波后位置误差（约 -3500~+3500） |
| `pid_turn_rpm` | `float` | 循迹 PID 差速输出 |
| `curvature_feedforward_rpm` | `float` | 右弯曲率前馈差速 |
| `turn_rpm` | `float` | 最终差速（PID + 前馈，已限幅） |
| `active_count` | `uint8` | 参与误差计算的有效探头数 |
| `right_active_count` | `uint8` | 右侧通道（默认 5~7）黑线数 |
| `right_curve_detected` | `uint8` | 右弯判定（连续 60 ms 且非横线） |
| `line_valid` | `uint8` | 本轮是否有有效探头 |
| `marker_detected` | `uint8` | 横线标志（≥6 路黑线） |
| `line_lost` | `uint8` | 丢线超时标志 |

| 函数 | 说明 |
| --- | --- |
| `line_control_init()` | 初始化循迹 PID |
| `line_control_reset()` | 复位 PID 与输出 |
| `line_control_set_base_rpm(base_rpm)` | 设置基准速度 |
| `line_control_update(values, now_ms, dt_s)` | 输入 8 路灰度快照，更新输出 |
| `line_control_get_output()` | 只读输出指针 |

误差语义：`LINE_SENSOR_ACTIVE_LEVEL=0` 用于位置 PID（补集误差）；`LINE_BLACK_ACTIVE_LEVEL=1` 用于右弯/横线/OLED 亮块。**两者勿混用。**

---

### 轮速内环 — `src/middle/wheel_speed_control.h`

#### 状态结构 — `wheel_speed_control_status_t`

| 字段 | 说明 |
| --- | --- |
| `left_target_rpm` / `right_target_rpm` | 目标 RPM |
| `left_measured_rpm` / `right_measured_rpm` | 编码器实测 RPM（已乘符号） |
| `left_duty` / `right_duty` | 下发 PWM duty |
| `left_saturated` / `right_saturated` | PID 是否饱和 |

| 函数 | 说明 |
| --- | --- |
| `wheel_speed_control_init()` | 初始化左右轮 PID |
| `wheel_speed_control_reset()` | 复位目标与 PID |
| `wheel_speed_control_set_target(left, right)` | 设置目标 RPM（clamp 至 `WHEEL_TARGET_RPM_LIMIT`） |
| `wheel_speed_control_update(period_ms, enabled)` | 更新测速与 PID；`enabled=0` 时停车 |
| `wheel_speed_control_get_status()` | 只读状态指针 |

内部调用 `encoder_update_speed(period_ms)` 与 `motor_set_speed()`。

---

### 通用 PID — `src/middle/control_pid.h`

带条件积分抗饱和的 PID 单元，供 `line_control` 与 `wheel_speed_control` 共用。

| 函数 | 说明 |
| --- | --- |
| `control_pid_init(pid, config)` | 初始化增益与限幅 |
| `control_pid_reset(pid)` | 清零积分与历史 |
| `control_pid_step(pid, error, feedforward, dt_s)` | 单步计算；返回限幅后输出 |

---

### 控制配置 — `src/middle/control_config.h`

仅宏定义，编译期配置。主要分组：

| 分组 | 代表宏 | 说明 |
| --- | --- | --- |
| 周期 | `CHASSIS_CONTROL_PERIOD_MS` | 10 ms |
| 车体 | `CHASSIS_WHEEL_TRACK_M`、`CHASSIS_WHEEL_DIAMETER_M` | 轮距 0.18 m、轮径 0.065 m |
| 电机接线 | `MOTOR_OUTPUT_SWAP_LEFT_RIGHT`、`MOTOR_*_OUTPUT_POLARITY` | 左右交换与极性 |
| 编码器符号 | `WHEEL_LEFT/RIGHT_ENCODER_SIGN` | 前进时 RPM 为正 |
| 轮速 PID | `WHEEL_LEFT/RIGHT_KP/KI/KD`、`WHEEL_*_KS/KV/KA` | 左右独立 |
| 轮速限幅 | `WHEEL_TARGET_RPM_LIMIT`（250）、`WHEEL_PWM_SLEW_DUTY_PER_S` | 目标 RPM 与 PWM 斜率 |
| PWM 映射 | `WHEEL_LEFT/RIGHT_PWM_MAP_SCALE` | 右轮默认 0.92 |
| 循迹 | `LINE_BASE_RPM_DEFAULT`（170）、`LINE_KP/KI/KD`、`LINE_ERROR_FILTER_ALPHA` | 外环增益与滤波 |
| 右弯 | `LINE_RIGHT_SENSOR_FIRST/LAST_INDEX`（5~7）、`LINE_RIGHT_CURVE_DETECT_MS`（60） | 右弯检测 |

符号检查流程见 [循迹两驱PID框架.md §7](../循迹两驱PID框架.md)。

---

## 电机模块（motor）

### 中间层 — `src/middle/motor.h`

#### 常量

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_SPEED_MAX` | 10000 | PWM duty 满量程 |
| `MOTOR_WATCHDOG_TIMEOUT_MS` | 100 | 看门狗超时 |

| 函数 | 说明 |
| --- | --- |
| `motor_init()` | 初始化 motor_hw、encoder，停止电机 |
| `motor_set_speed(left, right)` | 设置左右 duty；应用极性/交换（`control_config.h`） |
| `motor_brake()` / `motor_stop()` | 短路刹车 / 滑行停止 |
| `motor_watchdog_kick()` | **主循环每轮调用**；刷新看门狗 |
| `motor_watchdog_check()` | 超时则强制 `motor_stop()`（可选调用） |

非零 PWM 时看门狗武装；主循环停止超过 100 ms 则自动停车。

---

### 硬件层 — `src/hardware/motor_hw.h`

TB6612 **PWMA/PWMB + GPIO 方向** 接法：

| 信号 | 引脚 | TIM_A0 |
| --- | --- | --- |
| 左 PWMA | A0 | CH0 |
| 右 PWMB | A1 | CH1 |
| 左 AIN1 / AIN2 | B2 / B3 | — |
| 右 BIN1 / BIN2 | B4 / B5 | — |

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_HW_PWM_FREQ_HZ` | 17000 | PWM 频率 |
| `MOTOR_HW_DUTY_MAX` | 10000 | 占空比满量程 |

主要接口：`motor_hw_init()`、`motor_hw_set_duty()`、`motor_hw_brake()`、`motor_hw_stop_all()`。

---

## 编码器模块（encoder）

### 中间层 — `src/middle/encoder.h`

GM37-520 + 11 线编码器，减速比 30:1，QEI 4 倍频，输出轴每转 **1320** counts。

| 名称 | 值 |
| --- | --- |
| `ENCODER_COUNTS_PER_WHEEL_REV` | 1320 |
| `ENCODER_RAW_COUNT_MOD` | 65536 |

| 函数 | 说明 |
| --- | --- |
| `encoder_init()` | 初始化 QEI；由 `motor_init()` 调用 |
| `encoder_get_left/right_raw_count()` | 16 位原始计数 |
| `encoder_get_left/right_total_count()` | 环形差分累加 `int32` 总里程 |
| `encoder_clear_*_count()` / `encoder_clear_all_count()` | 清零 |
| `encoder_update_speed(period_ms)` | 差分换算 RPM；由 **`wheel_speed_control_update()`** 调用 |
| `encoder_get_left_rpm()` / `encoder_get_right_rpm()` | 最近一次更新后的 RPM |

RPM 换算：`rpm = Δcount × 60000 / (1320 × period_ms)`

### 硬件层 — `src/hardware/encoder_hw.h`

| 侧别 | 定时器 | A 相 | B 相 |
| --- | --- | --- | --- |
| 左轮 | TIMG8 | B10 | B11 |
| 右轮 | TIMG9 | B7 | B9 |

---

## 心跳模块（heartbeat）

### 应用层 — `src/app/heartbeat_app.h`

| 函数 | 说明 |
| --- | --- |
| `heartbeat_app_init()` | 启动 LED + UART0；发送 `BOOT OK\r\n` |
| `heartbeat_app_process()` | 处理 1 Hz 心跳；泵出 UART TX FIFO |

### 中间层 — `src/middle/heartbeat.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `HEARTBEAT_PERIOD_MS` | 1000 | 1 Hz（LED + `[hb]`） |

| 函数 | 说明 |
| --- | --- |
| `heartbeat_init()` | 初始化 hardware |
| `heartbeat_process()` | 周期到时发送 `[hb]`；调用 `heartbeat_hw_uart_tx_pump()` |
| `heartbeat_get_ms()` | SysTick 毫秒计数；供控制环与非阻塞定时 |

### 硬件层 — `src/hardware/heartbeat_hw.h`

| 函数 | 说明 |
| --- | --- |
| `heartbeat_hw_init(tick_period_ms)` | PA14 LED + SysTick 1 ms |
| `heartbeat_hw_uart_send_string(str)` | 非阻塞入队 TX FIFO |
| `heartbeat_hw_uart_tx_pump()` | 主循环泵出 UART0 发送 |
| `heartbeat_hw_uart_flush_blocking()` | 阻塞 flush（仅 boot） |
| `heartbeat_hw_get_ms()` / `heartbeat_hw_get_sequence()` | 毫秒 / 心跳序号 |
| `heartbeat_hw_get_drop_count()` | TX FIFO 溢出计数 |

---

## 循迹模块（grayscale）

八路数字灰度：**3 位地址 AD0~AD2 + 1 位 OUT**，非阻塞扫描，禁止主循环 busy-wait。

引脚：**AD0=A15，AD1=A16，AD2=A12，OUT=A13**。PA17 停用；PA18 为 BSL 脚，不可作 OUT。

### 应用层 — `src/app/grayscale_app.h`

| 函数 | 说明 |
| --- | --- |
| `grayscale_app_init()` | 调用 `grayscale_init()` |
| `grayscale_app_process()` | 推进扫描；每完成一轮且间隔 ≥1 s 发送 `[gs]` |

### 中间层 — `src/middle/grayscale.h`

| 函数 | 说明 |
| --- | --- |
| `grayscale_init()` | 初始化 GPIO 与状态机 |
| `grayscale_process()` | 主循环每轮调用，立即返回 |
| `grayscale_get_values()` | 最近一轮完整 8 路快照（0/1） |
| `grayscale_is_scan_ready()` | 是否至少完成过一轮 |
| `grayscale_take_scan_ready()` | 取走就绪标志（app 层用） |
| `grayscale_get_scan_sequence()` | 扫描轮次序号；控制环用于判新数据 |

整轮扫描完成后原子发布 `values[]`，避免控制/OLED 读到撕裂数据。

### 硬件层 — `src/hardware/grayscale_hw.h`

| 函数 | 说明 |
| --- | --- |
| `grayscale_hw_init()` | AD0~AD2 推挽，OUT 上拉；默认选通道 0 |
| `grayscale_hw_select_channel(ch)` | 写通道 0~7（先 clear 再 set） |
| `grayscale_hw_read_out()` | 读 OUT 0/1 |

---

## IMU 模块（imu）

**ATK-MS901M** 模块通过 **UART1**（PA8 TX / PA9 RX，115200）推送变长帧；MCU 侧 **DMA 块接收 + 主循环解析**。详见 `docs/datasheet/ATK-MS901M/` 与 [docs/log/2026-07-29-chassis-control-and-peripheral-refactor.md](../log/2026-07-29-chassis-control-and-peripheral-refactor.md)。

### 协议

读帧：`55 55 ID LEN DATA[LEN] SUM`（SUM 为从首字节至末数据字节之和低 8 位）

| ID | 内容 |
| --- | --- |
| `0x01` | 姿态角 Roll / Pitch / Yaw |
| `0x03` | 加速度 ax / ay / az |

### 应用层 — `src/app/imu_app.h`

| 函数 | 说明 |
| --- | --- |
| `imu_app_init()` | 初始化 IMU 并立即 `imu_hw_rx_enable()` |
| `imu_app_process()` | 调用 `imu_process()`；在线时 1 s 输出 `[imu]` |

配置宏（`imu_app.c`）：`IMU_APP_DEBUG_PERIOD_MS=1000`，`IMU_APP_WAIT_DEBUG_PERIOD_MS=2000`。

就绪条件：`imu_is_online()` → 同时收到姿态与加速度（`IMU_FLAG_ANGLE | IMU_FLAG_ACCEL`）。

### 中间层 — `src/middle/imu.h`

#### 标志

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `IMU_FLAG_ANGLE` | 0x01 | 姿态角就绪 |
| `IMU_FLAG_ACCEL` | 0x02 | 加速度就绪 |
| `IMU_STALE_TIMEOUT_MS` | 50 | 数据过期阈值 |

#### 数据结构

```c
typedef struct { float roll, pitch, yaw; } imu_angle_t;
typedef struct { float ax, ay, az; } imu_accel_t;
typedef struct {
    imu_angle_t angle;
    imu_accel_t accel;
    uint8       flags;
    uint32      angle_time_ms;
    uint32      accel_time_ms;
} imu_snapshot_t;
```

| 函数 | 说明 |
| --- | --- |
| `imu_init()` / `imu_process()` | 初始化 / 主循环解析 |
| `imu_get_angle()` / `imu_get_accel()` | 最近解码结果 |
| `imu_get_update_flags()` | 已收到类型位掩码 |
| `imu_get_snapshot(snapshot)` | 拷贝快照 |
| `imu_is_type_ready(flag)` | 指定类型是否至少收到一帧 |
| `imu_is_online()` | 姿态 + 加速度均就绪 |
| `imu_get_good/bad/ignored_frame_count()` | 帧统计 |

物理量：角度 raw × `180/32768`（°）；加速度 raw × `4g×9.80665/32768`（m/s²）。

### 硬件层 — `src/hardware/imu_hw.h`

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `IMU_HW_DMA_BLOCK_SIZE` | 64 | DMA 块大小 |
| `IMU_HW_DMA_BLOCK_COUNT` | 4 | 块数量 |
| `IMU_HW_TX_TIMEOUT_CYCLES` | 8000000 | TX 超时 |

| 函数 | 说明 |
| --- | --- |
| `imu_hw_init()` | UART1 FIFO + DMA RX |
| `imu_hw_rx_enable()` | 开启 DMA 接收 |
| `imu_hw_acquire_block(length)` | 取一块就绪 RX 数据 |
| `imu_hw_release_block(block)` | 归还块 |
| `imu_hw_write_frame(frame, len)` | 发送配置帧（带 TX 超时） |
| `imu_hw_get_overflow_count()` | DMA/队列溢出计数 |

---

## OLED 模块（oled）

GME12864-49（128×64 SSD1306），**I2C0 @ 400 kHz**，SCL=B0，SDA=B1，地址 0x3C。

### 应用层 — `src/app/oled_app.h`

| 函数 | 说明 |
| --- | --- |
| `oled_app_init()` | 初始化、静态标签、首屏刷新 |
| `oled_app_process()` | 增量更新；未就绪时 1 s 重试 |

仪表盘布局：

| 页 | 内容 | 刷新 |
| --- | --- | --- |
| 0 | 标题 `MSPM0G3519` | init 一次 |
| 1 | `Yaw:` + 整数航向 | 100 ms，变化时推送 |
| 3 | `L:` / `R:` + 左右 RPM | 100 ms，变化时推送 |
| 5~6 | 右弯指示（「转」+ 曲率字形） | 50 ms，变化时推送 |
| 7 | 八路循迹条（每路 16 px） | 50 ms，变化时推送 |

未检测到 I2C ACK 时 `oled_app_process()` 立即 return。

### 中间层 — `src/middle/oled.h`

| 函数 | 说明 |
| --- | --- |
| `oled_init()` / `oled_is_ready()` | 初始化 / 探测 |
| `oled_show_char()` / `oled_show_string()` / `oled_show_int()` | 文本与数值 |
| `oled_clear_page()` / `oled_clear_page_segment()` | 局部清除 |
| `oled_fill_page_bar(page, values, count, block_width)` | 循迹条直写页缓冲 |
| `oled_refresh()` / `oled_refresh_pages(start, end)` | I2C 推送 |

---

## 系统入口 — `src/main.c`

```c
#include "grayscale_app.h"
#include "heartbeat_app.h"
#include "heartbeat_hw.h"
#include "imu_app.h"
#include "motor.h"
#include "motor_app.h"
#include "oled_app.h"
#include "zf_common_clock.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    grayscale_app_init();
    motor_app_init();
    imu_app_init();
    oled_app_init();

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
        imu_app_process();
        grayscale_app_process();
        motor_app_process();
        heartbeat_app_process();
        oled_app_process();
    }
}
```

### `clock_init(uint32 clock)` — `zf_common_clock.h`

外设上电、`SYSCFG_DL_init()`（UART0/1/7、I2C0、DMA）、`interrupt_init()`。

---

## 串口协议

**UART0**：115200-8-N-1，TX=PA10，RX=PA11。发送经 TX FIFO 非阻塞入队，主循环 `heartbeat_hw_uart_tx_pump()` 泵出。

| 时机 | 报文格式 | 示例 |
| --- | --- | --- |
| 上电 | `BOOT OK\r\n` | `BOOT OK` |
| 每 1 s | `[hb] <序号>,<左RPM>,<右RPM>\r\n` | `[hb] 3,120,-118` |
| 每 1 s（扫描就绪） | `[gs] <序号>,v=<v0>…<v7>\r\n` | `[gs] 2,v=00110000` |
| 每 1 s（IMU 在线） | `[imu] <序号>,roll,pitch,yaw,ax,ay,az,bad=,ovf=\r\n` | 浮点，度与 m/s² |
| IMU 未在线 | `[imu] wait,f=0x??,good=,bad=,ovf=\r\n` | 2 s 周期 |
| 每 250 ms（控制活动） | `[ctl] mode,e=,n=,r=<右侧数><弯标志>,f=,t=L/R,m=L/R,u=L/R,s=L/R\r\n` | 见下表 |

#### `[ctl]` 字段

| 字段 | 说明 |
| --- | --- |
| `mode` | `motor_app_mode_enum` 数值 |
| `e` | 滤波后位置误差（整数） |
| `n` | 有效探头数 |
| `r` | 两位：右侧黑线数 + 右弯标志（0/1） |
| `f` | 曲率前馈差速 RPM |
| `t` | 左右目标 RPM |
| `m` | 左右实测 RPM |
| `u` | 左右 PWM duty |
| `s` | 左右 PID 饱和标志 |

- `[hb]` 序号由 SysTick ISR 递增；格式化用 `%u`/`%d`，避免 microlib `%lu` 乱码。
- `[gs]` 的 `v0`~`v7` 对应 X1~X8；`1`=黑线/检测到。
- IMU 通信走 **UART1**（A8/A9），与 UART0 独立。

---

## 调用示例

### 落地循迹（app 层）

```c
#include "motor_app.h"

void start_line_follow(void)
{
    motor_app_set_base_rpm(60.0f);
    motor_app_set_line_follow_enabled(1u);
}

void emergency_stop(void)
{
    motor_app_stop();
}
```

### 架空轮速标定

```c
motor_app_set_speed_test(50.0f, 50.0f);
```

### 顺时针 1 m 圆 demo

```c
motor_app_set_right_circle_demo(120.0f);
```

### 读取循迹输出（middle 层）

```c
#include "line_control.h"
#include "grayscale.h"

void control_tick(uint32 now_ms, float dt_s)
{
    if (grayscale_get_scan_sequence() != last_seq)
    {
        last_seq = grayscale_get_scan_sequence();
        line_control_update(grayscale_get_values(), now_ms, dt_s);
    }

    const line_control_output_t *out = line_control_get_output();
    if (0u == out->line_lost)
    {
        /* out->left_rpm, out->right_rpm */
    }
}
```

### 读取 IMU（middle 层）

```c
#include "imu.h"

void read_heading(void)
{
    imu_process();

    if (imu_is_online())
    {
        const imu_angle_t *angle = imu_get_angle();
        /* angle->yaw 单位：度 */
    }
}
```

### 非阻塞定时

```c
#include "heartbeat.h"

static uint32 last_ms;

void periodic_task(void)
{
    uint32 now = heartbeat_get_ms();
    if ((now - last_ms) >= 100u)
    {
        last_ms = now;
        /* 每 100 ms 执行一次 */
    }
}
```

### 调整调试周期

| 模块 | 文件 | 宏 |
| --- | --- | --- |
| 心跳 | `heartbeat.h` | `HEARTBEAT_PERIOD_MS` |
| 灰度 | `grayscale_app.c` | `GRAYSCALE_APP_DEBUG_PERIOD_MS` |
| IMU | `imu_app.c` | `IMU_APP_DEBUG_PERIOD_MS` |
| 控制 | `motor_app.c` | `MOTOR_APP_DEBUG_PERIOD_MS` |
| OLED | `oled_app.c` | `OLED_APP_TEXT/GS/RECOVERY_PERIOD_MS` |
| 控制增益 | `control_config.h` | 全部 `LINE_*`、`WHEEL_*` 宏 |

---

## 依赖与约束

1. 电机 PWM：**TIM_A0**（A0/A1）；方向 GPIO B2~B5；编码器 QEI：**TIMG8/TIMG9**。
2. **UART0/1/7、I2C0、DMA** 由 SysConfig 初始化；勿对这些串口再次调用逐飞 `uart_init()`。
3. **禁止阻塞延时**：主循环及传感器/IMU 路径不得 busy-wait。
4. 灰度 GPIO（A12/A13/A15/A16）运行时 `gpio_init`，不改 `M0G3519.syscfg`。
5. **OLED I2C 刷新**、**控制环**、**IMU 解析**均在主循环；禁止在 ISR 中调用。
6. 主循环须每轮调用 `motor_watchdog_kick()` 与 `heartbeat_hw_uart_tx_pump()`。
7. 灰度消费者须读 `grayscale_get_values()` 完整快照，并用 `scan_sequence` 判新数据。
8. 比赛前建议降低或关闭 `[ctl]`/`[gs]`/`[imu]` 文本遥测。

---

## 后续扩展（尚未实现）

- 轮速环 `kS/kV/kA` 实车标定（当前为 0）
- `marker_detected` 接入 A 点横线停车状态机
- IMU 航向辅助短时丢线预测
- S 曲线加速度规划（替代当前一阶斜率限制）
- 滚球摆杆前馈接口
