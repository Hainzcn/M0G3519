# 任务日志：六轴 IMU 串口模块接入

| 字段 | 内容 |
| --- | --- |
| 日期 | 2026-07-27 |
| 摘要 | 放弃软件阻塞 I2C 方案，按 hardware → middle → app 分层接入厂家六轴 IMU UART 协议；UART1(A8/A9) 中断收包 + 主循环非阻塞解析；UART0 输出 `[imu]` 调试 |

## 1. 任务背景

厂家 I2C 例程（`docs/IIC_Getdata -AXIS6`）使用 Timer 5 ms 中断 + GPIO 位操作 I2C 阻塞读取，CPU 占用高。串口手册（`docs/6轴数据手册(串口通信）.pdf`）表明模块已内置解算，可 **主动推送** 二进制帧，更适合本工程「主循环禁止阻塞」约束。

## 2. 变更概览

| 类别 | 文件 | 说明 |
| --- | --- | --- |
| SysConfig | `M0G3519.syscfg`、`keil/ti_msp_dl_config.c/h` | 新增 UART1：PA8 TX / PA9 RX，115200 |
| 硬件层 | `src/hardware/imu_hw.c/h` | RX 中断 → FIFO；init 写寄存器命令（带 TX 超时） |
| 中间层 | `src/middle/imu.c/h` | 11 字节帧 FSM、校验、换算、`IMU_ENABLE_*` 宏 |
| 应用层 | `src/app/imu_app.c/h` | 非阻塞 boot 等待、可选 Yaw 归零、`[imu]` 调试 |
| 主程序 | `src/main.c` | 接入 `imu_app_init/process` |
| 工程 | `keil/.eide/eide.yml` | 注册 IMU 源文件 |
| 文档 | `docs/pin/pin.md` | 接线与协议说明 |

## 3. 引脚分配

| 模块信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| RX | A8 (PA8) | MCU UART1 TX → 模块 RX |
| TX | A9 (PA9) | 模块 TX → MCU UART1 RX |
| VCC | 5V | 共地 |

与现有外设无冲突（UART0 心跳 A10/A11 独立）。

## 4. 协议与数据流

**读帧（11 字节）**：`0x5A | TYPE | 8×data | SUM`

| TYPE | 内容 |
| --- | --- |
| 0xAA | 角速度 (±2000°/s) |
| 0xBB | 姿态角 Roll/Pitch/Yaw (±180°) |
| 0xCC | 加速度 (±16g) |
| 0xDD | 四元数 |

**写帧（5 字节，仅 init）**：`55 AA ADDR DATAL DATAH` — 解锁 KEY(0x13)、SAVE(0x00)、Yaw 归零(0x0A) 等。

```text
模块 UART TX ──► UART1 RX ISR ──► FIFO ──► imu_process() FSM ──► 缓存
                                                      │
UART0 ◄── heartbeat_hw ◄── imu_app [imu] 调试（200 ms）
```

## 5. 开关宏

在 `src/middle/imu.h`：

```c
#define IMU_ENABLE_GYRO   (1)
#define IMU_ENABLE_ANGLE  (1)
#define IMU_ENABLE_ACCEL  (1)
#define IMU_ENABLE_QUAT   (1)
```

设为 `0` 时跳过对应 TYPE 解码（帧仍消费以保持流同步）；`imu_app` 调试字段同步裁剪。

在 `src/app/imu_app.c`：`IMU_APP_YAW_ZERO_ON_BOOT` 默认 `0`；设为 `1` 时上电执行 Yaw 归零。

## 6. 联调步骤

1. 接线：模块 TX→A9，模块 RX→A8，5V/GND 共地
2. 烧录后 UART0（115200）应先见 `BOOT OK`、`[hb]`
3. 约 500 ms 后 IMU 接收就绪；模块接好后周期性出现 `[imu]`（数值 ×100 或 ×1000，见下文）
4. 无 `[imu]`：查 TX/RX 交叉、115200、模块波特率是否被改过
5. 将 `IMU_ENABLE_QUAT/ACCEL` 设为 0 后 rebuild，确认 `[imu]` 字段缩短

`[imu]` 串口为定点整数：角度/角速度/加速度字段 ÷100 得物理量，四元数字段 ÷1000。

## 7. 后续扩展

- 提高输出速率：unlock → 写 RRATE(0x02) → SAVE（见手册）
- 陀螺仪校准：`imu_app_calibrate()` 可封装 BIAS_CAL 流程（6 s 阻塞，仅显式调用）
- 闭环控制：直接读 `imu_get_angle()` / `imu_get_gyro()`

## 8. 故障修复（2026-07-28）

**现象**：烧录后状态灯常亮，UART0 无 `BOOT OK` / `[hb]`。

**根因**：首版 `imu_app_init()` 在进入主循环前阻塞约 800 ms；Yaw 归零时 `imu_hw_write_frame()` 对 `DL_UART_isBusy` 无超时，IMU 未接或 TX 异常时会永久阻塞，心跳与 LED 翻转均无法执行。另：主栈仅 256 B，后续 `snprintf` 浮点格式化可能导致栈溢出。

**修复**：

- init 非阻塞；500 ms 等待与 Yaw 归零改为 `imu_app_process()` 状态机
- UART1 TX 超时；RX 中断在 boot 等待后由 `imu_hw_rx_enable()` 开启
- `IMU_APP_YAW_ZERO_ON_BOOT` 默认 `0`
- 主栈 256 B → 2 KB；`[imu]` 改为整数（×100 或 ×1000）输出