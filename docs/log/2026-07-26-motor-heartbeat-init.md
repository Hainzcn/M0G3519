# 任务日志：电机驱动与心跳串口基础搭建

| 字段 | 内容 |
| --- | --- |
| 提交 | `a74c9502b3b2b72fd1d8a768c3499f07e80030fc` |
| 作者 | Hainzcn |
| 日期 | 2026-07-26 |
| 摘要 | 添加 UART0 配置和初始化，更新文档以包含电机驱动和编码器信息，新增调试和 FIFO 相关功能，优化时钟管理 |

## 1. 任务目标

为 MSPM0G3519 循迹小车项目完成第一阶段固件搭建：

1. 按 **hardware → middle → app** 分层实现 TB6612FNG 双电机开环驱动。
2. 接入逐飞 MSPM0G3519 开源库（复制至 `src/MSPM0G3519_Library`）。
3. 实现程序状态指示灯（PA14）与 UART0 心跳串口（PA10/PA11）。
4. 补充引脚接线文档，预留编码器占位信息。

## 2. 架构与目录

```
src/
├── main.c                 # 入口：clock_init → heartbeat → motor → 主循环
├── hardware/              # 外设与引脚
│   ├── motor_hw.c/h       # TB6612 双 PWM
│   └── heartbeat_hw.c/h   # LED + UART0 + SysTick
├── middle/                # 业务抽象
│   ├── motor.c/h          # 双轮速度接口（开环）
│   └── heartbeat.c/h      # 心跳周期与报文
├── app/                   # 应用入口
│   ├── motor_app.c/h
│   └── heartbeat_app.c/h
└── MSPM0G3519_Library/    # 逐飞库（工程实际编译使用）
```

依赖关系：`app → middle → hardware → 逐飞库 / DriverLib`。

## 3. 完成内容

### 3.1 电机驱动（TB6612FNG）

- **硬件层** `motor_hw`：四路 PWM 均落在 `TIM_A0` CH0~CH3。
  - 左轮：A0/A1（AIN1/AIN2）
  - 右轮：B12/B13（BIN1/BIN2）
  - PWM 频率 17 kHz，占空比量程 0~10000，符号表示转向。
  - 支持正转、反转、滑行停止（占空比 0）、短路刹车（双臂满占空比）。
- **中间层** `motor`：提供归一化双轮速度接口 `motor_set_speed(left, right)`，当前为开环直通映射。
- **应用层** `motor_app`：上电初始化后默认静止；提供 `motor_app_demo()` 供联调（正转 → 反转 → 停止），默认不在 `main` 中调用。
- **编码器**：引脚占位（左 B10/B11/B9，右 A26/B27/A27）已写入 `docs/pin/pin.md`，代码中尚未初始化。

### 3.2 心跳与状态指示

- **硬件层** `heartbeat_hw`：
  - PA14 板载 LED，推挽输出，默认熄灭。
  - UART0 TX/RX：PA10/PA11，115200-8-N-1。
  - SysTick 1 ms 中断累积周期标志；UART 发送在主循环中执行，避免在中断里阻塞。
- **中间层** `heartbeat`：500 ms 周期翻转 LED 并发送 `HEARTBEAT,<计数值>\r\n`；上电立即发送 `BOOT OK\r\n`。
- **应用层** `heartbeat_app`：提供 `heartbeat_app_init()` / `heartbeat_app_process()`，由 `main` 主循环调用。

### 3.3 UART0 与 SysConfig

- 在 `M0G3519.syscfg` 中新增 UART0 实例：
  - 引脚 PA10（TX）/ PA11（RX）
  - 波特率 115200，`uartClkDiv = 8`
- 重新生成 `keil/ti_msp_dl_config.c/h`，由 `SYSCFG_DL_init()` 完成 UART 引脚复用与波特率配置。
- 心跳发送使用 TI DriverLib `DL_UART_Main_transmitDataBlocking()`，**不再调用**逐飞 `uart_init()`，避免覆盖 SysConfig 生成的 UART 参数。

### 3.4 时钟与库集成

- `main()` 改为调用 `clock_init(SYSTEM_CLOCK_80M)`，内部包含：
  - 外设上电复位（含 UART0）
  - `SYSCFG_DL_init()`（80 MHz PLL + UART0 初始化）
  - `interrupt_init()`
- 将逐飞库完整复制到 `src/MSPM0G3519_Library`；工程编译链接以下模块：
  - `zf_common_clock`、`zf_common_debug`、`zf_common_fifo`、`zf_common_interrupt`
  - `zf_driver_delay`、`zf_driver_gpio`、`zf_driver_pwm`、`zf_driver_timer`、`zf_driver_uart`
- 更新 `keil/.eide/eide.yml` 虚拟文件夹、源文件列表与 include 路径。

### 3.5 文档

- 更新 `docs/pin/pin.md`：TB6612 接线、编码器占位、心跳串口与 SWD/XDS110/CH340 接线说明、库路径说明。

## 4. 主程序流程

```c
int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);   // SysConfig + 外设上电

    heartbeat_app_init();           // 先启动串口，便于联调
    motor_app_init();               // 电机默认停止

    while (1)
    {
        heartbeat_app_process();    // 处理待发送心跳
    }
}
```

## 5. 联调与验证

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| Keil/EIDE 编译链接 | 通过 | armclang + armlink 验证 |
| PA14 LED 500 ms 翻转 | 待板端确认 | SysTick + 主循环处理 |
| UART0 上电 `BOOT OK` | 待板端确认 | 115200，PA10/PA11 或 XDS110 User UART |
| 心跳 `HEARTBEAT,n` | 待板端确认 | 每 500 ms 一条 |
| 电机开环驱动 | 待板端确认 | 可手动调用 `motor_app_demo()` 验证 |

## 6. 已知设计约束

1. **UART 初始化路径**：必须使用 `clock_init()` → `SYSCFG_DL_init()`，不可单独调用逐飞 `uart_init()` 覆盖 UART0 配置。
2. **定时器资源**：电机 PWM 占用 `TIM_A0`；心跳节拍使用 SysTick，不占用通用定时器。
3. **中断分工**：SysTick 仅置标志；LED 翻转与串口发送均在主循环完成。
4. **编码器**：占位引脚已文档化，闭环接口预留于 `motor.h` 注释，尚未实现。

## 7. 后续计划

- [x] 板端确认 UART 与 LED 心跳表现。
- [ ] 确认 TB6612 正反转方向，必要时在 `motor_hw` 或 `motor` 层增加极性修正。
- [ ] 核对编码器实际接线后，在 `middle/motor.c` 接入 `encoder_quad_init` 与速度闭环。
- [ ] 循迹传感器模块接入与 app 层任务调度。
