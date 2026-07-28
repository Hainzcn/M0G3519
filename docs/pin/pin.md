# MSPM0G3519 CoreBoard V1.0 引脚排布总览
| 左外 | 左内 |        中间（MSPM0G3519S）        | 右外 | 右内 |
| ---- | ---- | -------------------------------- | ---- | ---- |
| A2   | A7   |                                  | A31  | A30  |
| B0   | B1   |                                  | A29  | A28  |
| B2   | B3   |                                  | A1   | A0   |
| B4   | B5   |                                  | A27  | A26  |
| A8   | A9   |                                  | B27  | B26  |
| A10  | A11  |                                  | B25  | A25  |
| B6   | B7   |                                  | A24  | A23  |
| B8   | B9   |                                  | B24  | B23  |
| B10  | B11  |             PA14 LED             | B22  | B21  |
| B12  | B13  |                                  | B20  | A22  |
| B14  | B15  |      RST按键　　　　BSL按键       | A21  | B19  |
| B16  | A12  |                                  | B18  | B17  |
| A13  | A14  | SWD：RST RX TX NC CLK DIO GND 3V3| 3V3  | 3V3  |
| A15  | A16  |                                  | 5V   | 5V   |
| A17  | A18  |                                  | GND  | GND  |

# 接线表

## TB6612FNG 电机驱动

### 芯片控制引脚（必读）

| 芯片引脚 | 作用 |
| --- | --- |
| **AIN1 / AIN2** | A 路方向输入 → 输出 **AO1 / AO2** 接左电机 |
| **PWMA** | A 路独立 PWM 输入，芯片内置下拉 |
| **BIN1 / BIN2** | B 路方向输入 → 输出 **BO1 / BO2** 接右电机 |
| **PWMB** | B 路独立 PWM 输入，芯片内置下拉 |
| **STBY** | 高电平使能；接 **3.3V**（与 VCC 一致） |
| **VM** | 电机主电源 5~12V（与 MCU 3.3V 分离，GND 共地） |

PWMA/PWMB 是芯片的独立输入，不能与 AINx/BINx 混用。若 PWMA/PWMB 未接，会被内部下拉，AO/BO 均为高阻，即使方向脚电平正确也不会驱动电机。

**TB6612 真值表（STBY=高，PWM=高时）**

| IN1 | IN2 | 输出 |
| --- | --- | --- |
| L | L | 短路刹车 |
| H | L | 正转 |
| L | H | 反转 |
| H | H | 短路刹车 |

PWM 为低时，输出为高阻，电机滑行。

### 本工程接线（标准 PWM + GPIO 方向）

| 模块/芯片 | MCU 引脚 | TIM_A0 | 运行时角色 |
| --- | --- | --- | --- |
| PWMA | A0 | CH0 PWM | 左电机调速 |
| PWMB | A1 | CH1 PWM | 右电机调速 |
| AIN1 | B2 | - | 左电机方向 GPIO |
| AIN2 | B3 | - | 左电机方向 GPIO |
| BIN1 | B4 | - | 右电机方向 GPIO |
| BIN2 | B5 | - | 右电机方向 GPIO |
| STBY / VCC | 3.3V | - | 逻辑电源 |
| VM | 5~12V | - | 电机电源 |

方向 GPIO 使用排针中连续的 2×2 区域 `B2/B3/B4/B5`。该分配避开 A30/A31、B0/B1 用户按键，并保留 B10/B11（左轮 TIMG8）和 B7/B9（右轮 TIMG9）的 QEI 连接。

控制逻辑（`src/hardware/motor_hw.c`）：

- **正转**：IN1=高，IN2=低，PWM=目标占空比。
- **反转**：IN1=低，IN2=高，PWM=目标占空比。
- **停止**：PWM=低（输出高阻滑行）。
- **刹车**：PWM=高，IN1=IN2=低。

宏定义见 `motor_hw.h`：`MOTOR_HW_LEFT_PWMA_PWM`、`MOTOR_HW_LEFT_AIN1_GPIO` 等。

### 联调：万用表预期（正转、占空比 70%）

| 测量点 | 预期 |
| --- | --- |
| PWMA | PWM 平均 ~2.3V |
| AIN1 / AIN2 | 约 3.3V / 0V |
| AO1–AO2 | **VM 已接** 时有电压；恒 0 → 查 VM 或芯片 |

## 增量式编码器（正交 QEI，已实现）

MSPM0G3519 上逐飞库 `encoder_quad_init()` **仅支持 `TIM_G8` 与 `TIM_G9` 两路硬件 QEI**，正好对应双轮各一路，与电机 PWM 占用的 `TIM_A0` 互不冲突。

正交（AB 相）解码 **只需 2 根信号线**（A 相 + B 相），外加编码器 VCC/GND。原先文档写「B10/B11/**B9**」「A26/**B27**/A27」中的第三根引脚是早期占位：B9、B27 可能是带 Index(Z) 的编码器第三相，或排针位置备忘；**QEI 正交模式不需要接入 MCU**。

| 侧别 | QEI 定时器 | A 相 | B 相 | 库内宏定义 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 左轮 | **TIMG8** | B10 | B11 | `TIMG8_ENCODER1_CH1_B10` / `TIMG8_ENCODER1_CH2_B11` | 与库示例一致 |
| 右轮 | **TIMG9** | B7 | B9 | `TIMG9_ENCODER1_CH1_B7` / `TIMG9_ENCODER1_CH2_B9` | 见下方引脚冲突说明 |

### 关于 A26 / A27 / B27

| 引脚 | 能否用于 QEI | 说明 |
| --- | --- | --- |
| A26 | 仅 **TIMG8** CCP0（A 相通道） | 与左轮 B10/B11 同占 TIMG8，**不能**与左轮同时再开一路 QEI |
| A27 | 仅 **TIMG8** CCP1（B 相通道） | 与 A26 配对可组成一路 TIMG8 QEI，但与左轮冲突 |
| B27 | **不支持 QEI** | 无 TIMG8/TIMG9 正交复用，若为编码器 Index(Z) 可不接 MCU |

若右轮编码器物理上接在 **A26/A27**，而左轮已在 **TIMG8（B10/B11）**，则无法同时启用两路正交 QEI。可选方案：

1. **推荐**：右轮 AB 改接到 **B7（A 相）+ B9（B 相）**，走 TIMG9（当前代码默认方案）。
2. 左轮改线到 TIMG9 可用引脚、右轮用 A26/A27 走 TIMG8——但 **B10/B11 无法复用到 TIMG9**，左轮也必须换线，一般不采纳。

A/B 相在软件里只区分「接 CCP0 还是 CCP1」；接反了只会导致计数方向相反，可在中间层对 `encoder_get_*_count()` 取反修正，**不必固定某一物理线必须叫 A 或 B**。

### 电机与转速换算（GM37-520）

| 参数 | 值 |
| --- | --- |
| 电机型号 | GM37-520 |
| 编码器线数 | 11 线（电机轴，单通道脉冲数） |
| 减速比 | 30:1 |
| QEI 倍频 | ×4（正交 AB 相硬件解码） |
| 输出轴每转计数 | 11 × 4 × 30 = **1320** counts/rev |

逐飞库仅提供 `encoder_get_count()` 原始计数，**无现成 RPM API**；`src/middle/encoder.c` 在心跳周期内对 16 位原始计数做**环形差分**（`(int16)(now - last)`），累加到 `int32` 总里程，并换算为输出轴转速（RPM）：

`rpm = Δcount × 60000 / (1320 × period_ms)`

硬件 16 位计数约每 50 圈（输出轴）回绕一次；中间层差分/累加已处理回绕，可长时间运行。清零调用 `encoder_clear_*_count()`。

实现见 `src/hardware/encoder_hw.c`（`encoder_quad_init`）、`src/middle/encoder.c`（`encoder_update_speed` / `encoder_get_*_rpm` / `encoder_get_*_total_count`）；`motor_init()` 内已调用 `encoder_init()`，当前仍开环驱动，转速供监控与后续闭环使用。

## 程序状态指示灯与心跳串口（已实现）

| 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| 状态灯 | A14 | 核心板板载 LED，推挽输出；上电默认熄灭，1 Hz 翻转（每 1s 一次） |
| UART0 TX | A10 | 心跳报文发送，115200-8-N-1，由 SysConfig 初始化 |
| UART0 RX | A11 | 预留（当前心跳功能不接收数据） |

串口接线说明（MCU 侧均为 UART0，引脚固定为 PA10/PA11）：

| 连接方式 | 说明 |
| --- | --- |
| 核心板排针 A10/A11 | PA10 为 MCU 发送端，需接 USB-TTL 的 **RX**；PA11 接 USB-TTL 的 **TX**；必须共地 |
| 板底 SWD 排针 RX/TX | 这是 XDS110 下载器转出的 UART0，**不是** PA13/PA14；下载器内部已接到 PA10/PA11。使用 XDS110 时请打开设备管理器里带 **Application/User UART** 字样的 COM 口 |
| IO 底板 USB-C（若有 CH340） | 同样复用 UART0（PA10/PA11），波特率请设为 **115200**（本工程固定 115200，不是部分例程默认的 9600） |

上电后应先收到 `BOOT OK`，之后每 1s 收到 `[hb],<序号>,<左转速>,<右转速>`（单位 RPM，输出轴），例如 `[hb],1,120,-118`。若 LED 正常闪烁但串口仍无数据，优先检查 COM 口选择与波特率，其次确认 TX/RX 是否交叉、是否共地。

节拍来源使用 Cortex-M0+ 的 `SysTick` 1ms 中断，不占用电机 PWM 使用的 `TIM_A0` 或其他通用定时器。中断中只累积待处理标志；主循环调用 `heartbeat_app_process()` 后才翻转状态灯并通过 UART0 发送心跳报文。`main()` 必须通过 `clock_init()` 完成外设上电，否则 UART0 无法工作。

## 八路循迹模块（已实现）

模块为 **3 位地址（AD0~AD2）+ 1 位数字输出（OUT）** 的多路复用结构，MCU 依次写入通道号 0~7 并读取 OUT 电平（0/1）。厂家例程使用 PA14~PA17，本工程 **A14 已作状态灯**，故地址线整体偏移至 A15~A17。

| 模块信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| AD0 | A15 (PA15) | 输出推挽 | 地址 bit0 |
| AD1 | A16 (PA16) | 输出推挽 | 地址 bit1 |
| AD2 | A17 (PA17) | 输出推挽 | 地址 bit2 |
| OUT | A18 (PA18) | 输入上拉 | 数字输出；上拉兼容开漏型模块 |
| VCC | 5V | — | 模块供电（与厂家例程一致） |
| GND | GND | — | 与 MCU 共地 |

读取逻辑见 `src/hardware/grayscale_hw.c`（GPIO）、`src/middle/grayscale.c`（非阻塞扫描状态机）。地址切换后约 50 µs 稳定时间通过 **只读 SysTick VAL 轮询** 实现，不在主循环中使用阻塞延时。`grayscale_process()` 在主循环中分步推进，每轮扫描 8 路约 400 µs 物理时间，分散在多次循环迭代中完成。

串口调试：上电后除 `[hb]` 心跳外，每 500 ms 输出一路 `[gs]` 报文，格式为：

`[gs],<序号>,v0,v1,v2,v3,v4,v5,v6,v7`

其中 `v0`~`v7` 对应 X1~X8 探头，值为 0 或 1（厂家说明：灯亮/检测到为 1）。传感器联调阶段建议将 `motor_app.c` 中 `MOTOR_APP_DEMO_ENABLE` 设为 `0`，避免电机转动干扰读数。

## 单轴 IMU 模块（UART，已实现）

模块内置 Z 轴姿态解算，通过 **UART 主动推送** Z 轴角速度与航向角（Yaw）二进制帧；MCU 使用 **UART1** 接收，**UART0** 仍专用于心跳/调试输出。

| 模块信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| VCC | 5V | — | 手册典型 5V（3.3~16V） |
| GND | GND | — | 与 MCU 共地 |
| RX | A8 (PA8) | MCU → 模块 | UART1 TX，TTL |
| TX | A9 (PA9) | 模块 → MCU | UART1 RX，TTL |

通信参数：**115200-8-N-1**（与模块出厂默认一致）。读帧格式见 `docs/数据手册(串口通信).pdf`：**5 字节** `0x5A | TYPE | DATAL | DATAH | SUM`。

| TYPE | 内容 |
| --- | --- |
| 0xAA | Z 轴角速度 Wz（±400°/s 量程） |
| 0xBB | 航向角 Yaw（±180°） |

软件分层：

- `src/hardware/imu_hw.c/h` — UART1 RX 中断 + 512B FIFO；写寄存器命令（带 TX 超时）
- `src/middle/imu.c/h` — 5 字节帧状态机、校验、物理量换算
- `src/app/imu_app.c/h` — 上电 500 ms 等待；可选 Yaw 归零（`IMU_APP_YAW_ZERO_ON_BOOT`，寄存器 0x15）；主循环 `imu_process()`

串口调试（UART0）：除 `[hb]` / `[gs]` 外，每 1 s 输出 `[imu]` 行：

`[imu],<序号>,yaw,wz`

单位分别为 **°** 与 **°/s**（保留两位小数）。就绪需同时收到 0xAA 与 0xBB 帧；等待时为 `[imu],0,wait,flags=0x??`。

联调注意：模块 TX 必须接 MCU A9（RX），A8 接模块 RX；若仅有 `[hb]` 无 `[imu]`，检查交叉接线、115200 波特率及模块是否被改为其他波特率。

## 第三方库位置

`@docs/MSPM0G3519_Library` 中的开源库已复制一份到 `src/MSPM0G3519_Library`，工程实际编译使用 `src/` 下的这份拷贝（详见 `keil/.eide/eide.yml`）。`docs/MSPM0G3519_Library` 保留作为原始参考，两者内容目前一致；后续如需升级库版本，请只更新 `src/MSPM0G3519_Library` 并同步说明，避免两份代码长期不一致。
