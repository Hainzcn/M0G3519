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

逻辑左右轮与实际接线可在 `control_config.h` 中校准：`MOTOR_OUTPUT_SWAP_LEFT_RIGHT` 控制左右通道交换，`MOTOR_LEFT_OUTPUT_POLARITY`、`MOTOR_RIGHT_OUTPUT_POLARITY` 分别控制正反极性。极性只允许使用 `1` 或 `-1`；编码器方向仍由独立的 `WHEEL_LEFT/RIGHT_ENCODER_SIGN` 校准。

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

## Emm42 步进电机驱动（UART7，已实现）

Emm42 使用独立硬件串口，不占用 UART0 心跳/调试或 UART1 IMU。

| Emm42 信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| MCU 侧 RX / RAH | A23 (PA23) | MCU → PLC 驱动接口 | UART7 TX，115200-8-N-1 |
| MCU 侧 TX / TBL | A24 (PA24) | PLC 驱动接口 → MCU | UART7 RX，115200-8-N-1 |
| GND | GND | — | MCU、驱动器必须共地 |

PA23/PA24 当前未被 PWM、QEI、灰度、OLED、IMU 或调试串口占用。它们只接 PLC 驱动电路的 MCU 逻辑侧，不得绕过隔离/电平转换后连接 24V PLC 端子。电机和驱动器使用独立电源，不得由核心板供电。

## 蓝牙串口模块（UART3，测试模式）

蓝牙模块使用独立 UART3，不占用 UART0 调试、UART1 IMU 或 UART7 Emm42。

| 蓝牙模块信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| RX | B13 (PB13) | MCU -> 蓝牙模块 | UART3 TX，115200-8-N-1 |
| TX | B12 (PB12) | 蓝牙模块 -> MCU | UART3 RX，115200-8-N-1 |
| GND | GND | - | MCU 与蓝牙模块必须共地 |

模块供电电压按具体型号的数据手册选择；UART 信号电平必须兼容 3.3V TTL，禁止将 5V 串口信号直接接入 MCU。当前固件上电进入蓝牙测试模式：模块会收到 `[bt] ready,115200`，随后每秒收到 `[bt] alive`；从蓝牙端发送的任意字节会由 MCU 原样回显。测试期间电机保持停机，循迹和各 demo 均不运行。

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

## 八路红外循迹模块（I2C，已实现）

新模块独占硬件 I2C1 的 400 kHz 快速模式，7 位地址为 `0x12`。读取寄存器 `0x30` 可一次获得全部八路数字量：X1 对应 bit7，X8 对应 bit0。独立控制器避免 OLED 刷新占用循迹总线。

| 模块信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| SCL | A15 (PA15) | MCU -> 模块 | I2C1_SCL，PF4 |
| SDA | A16 (PA16) | 双向 | I2C1_SDA，PF4 |
| VCC | 5V | — | 与官方例程接线一致 |
| GND | GND | — | 与 MCU、OLED 共地 |

原灰度模块的 AD0/AD1/AD2/OUT 接线已停用，A12/A13 释放；A15/A16 改作红外模块 I2C1。模块虽然按官方说明使用 5V 供电，但 **SDA/SCL 高电平必须实测不超过 3.3V**；若模块板将 I2C 上拉到 5V，必须移除该上拉并改接 3.3V，或增加双向电平转换器。

读取逻辑见 `src/hardware/i2c_bus.c`（双控制器非阻塞事务）、`src/hardware/grayscale_hw.c`（I2C1 `0x30` 寄存器事务）和 `src/middle/grayscale.c`（位图转换与发布）。循迹以 2 ms 周期请求快照，主循环不使用阻塞延时；I2C1 中断优先级高于 OLED 的 I2C0。

### 硬件 I2C 引脚复用检查

下表按数据手册的自然成对引脚列出。一个控制器的 SCL/SDA 也可从各自候选中重新组合，但应优先选同排针相邻引脚。

| 控制器 | 常用硬件引脚对（SCL/SDA） | 当前结论 |
| --- | --- | --- |
| I2C0 | PA1/PA0 | 冲突：电机 PWM |
| I2C0 | PA9/PA8 | 冲突：IMU UART1 |
| I2C0 | PA11/PA10 | 冲突：调试 UART0 |
| I2C0 | PA31/PA28 | 可用，排针距离较远 |
| I2C0 | PB0/PB1 | **已释放，可供后续外设使用** |
| I2C0 | PB17/PB18 | **OLED 当前使用** |
| I2C0 | PB21/PB22 | 可用；PB20 也是 I2C0_SDA 候选 |
| I2C1 | PA4/PA3、PA6/PA5 | 芯片支持，当前核心板排针未引出 PA3~PA6 |
| I2C1 | PA11/PA10 | 冲突：调试 UART0 |
| I2C1 | PA15/PA16 | **红外循迹当前使用** |
| I2C1 | PA17/PA18 | 不采用：PA18 是 BSL 启动采样脚 |
| I2C1 | PA20/PA19 | 不采用：SWD 调试口 |
| I2C1 | PA29/PA30 | 可用 |
| I2C1 | PB2/PB3 | 冲突：电机方向 |
| I2C2 | PA15/PA16 | 冲突：红外 I2C1 |
| I2C2 | PA23/PA24 | 冲突：Emm42 UART7 |
| I2C2 | PA29/PA30 | 可用（与 I2C1 候选复用冲突） |
| I2C2 | PB6/PB7、PB8/PB9 | 冲突：PB7/PB9 为右轮 QEI |
| I2C2 | PB15/PB16 | 可用 |

串口调试：上电后无论模块是否在线，每 1 s 输出一路 `[gs]` 报文，格式为：

`[gs] <序号>,on=<0/1>,raw=<HEX>,v=<v0>...<v7>,err=<错误数>,age=<数据年龄ms>`

例如 `[gs] 12,on=1,raw=3C,v=00111100,err=0,age=1`。OLED 第 2 页同步显示 `IR:<raw>` 与 `E:<错误数>`；离线时显示 `IR:OFF`，底部第 7 页为八路循迹带。

其中 `v0`~`v7` 对应 X1~X8 探头，值为 0 或 1（厂家说明：灯亮/检测到为 1）。传感器联调阶段建议将 `motor_app.c` 中 `MOTOR_APP_DEMO_ENABLE` 设为 `0`，避免电机转动干扰读数。

## ATK-MS901M IMU 模块（UART，已实现）

模块通过 **UART 主动推送** 姿态角和加速度数据；MCU 使用 **UART1** 接收，**UART0** 仍专用于心跳/调试输出。当前模块已经设置为 **200Hz、115200-8-N-1**，程序不会在上电时修改或保存模块配置。

| 模块信号 | MCU 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| VCC | 5V | — | 模块支持 3.3V/5V，手册推荐 5V |
| GND | GND | — | 与 MCU 共地 |
| RX | A8 (PA8) | MCU → 模块 | UART1 TX，TTL |
| TX | A9 (PA9) | 模块 → MCU | UART1 RX，TTL |

通信参数：**115200-8-N-1，200Hz**。主动上报帧格式见 `docs/ATK-MS901M/ATK-MS901M模块用户手册_V1.0.pdf`：

`0x55 | 0x55 | ID | LEN | DATA[LEN] | SUM`

`SUM` 为校验和之前所有字节相加的低 8 位。

| ID | 长度 | 内容 | 本工程处理 |
| --- | --- | --- | --- |
| 0x01 | 6 | Roll/Pitch/Yaw，三个 int16 小端 | 全部换算为 ° |
| 0x03 | 12 | Ax/Ay/Az/Gx/Gy/Gz，六个 int16 小端 | 只换算前三轴加速度 |

当前只开启 0x01 和 0x03 时，每周期共 28 字节，200Hz 约 5600B/s，占 115200-8-N-1 有效链路能力约 49%。如果模块还开启了四元数、磁力计或气压计上报，应先关闭多余上报项，否则 115200 波特率无法可靠承载所有数据。

加速度换算默认模块量程为官方默认 **±4g**：

`accel_mps2 = raw / 32768 × 4 × 9.80665`

若模块量程已被改为 ±2g/±8g/±16g，必须同步修改 `IMU_ACCEL_FSR_G`。

软件分层：

- `src/hardware/imu_hw.c/h` — UART1 RX 中断 + 512B 环形缓冲；ISR 只收字节
- `src/middle/imu.c/h` — 流式状态机、批量取数、校验、姿态角/加速度换算和 50ms 新鲜度检查
- `src/app/imu_app.c/h` — 立即开启接收；1Hz 限流输出状态，不发送模块配置命令

串口调试（UART0）：除 `[hb]` / `[gs]` 外，每 1 s 输出 `[imu]` 行：

`[imu] seq,roll,pitch,yaw,ax,ay,az,bad=N,ovf=N`

姿态角单位为 **°**，加速度单位为 **m/s²**。就绪要求 50ms 内同时收到 0x01 和 0x03；等待输出还会显示有效标志、正确帧数、坏帧数和环形缓冲溢出数。

联调注意：模块 TX 必须接 MCU A9（RX），A8 接模块 RX；若仅有 `[hb]` 无 `[imu]`，检查交叉接线、115200 波特率及模块是否被改为其他波特率。

## OLED 显示模块（GME12864-49，已实现）

0.96 寸 **128×64** 单色 OLED，4 针 I2C（控制器 SSD1306/兼容），使用独立的 **硬件 I2C0**（400 kHz，SysConfig 初始化）。原 B0/B1 接线已迁移并释放。

| 模块信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| VCC | 3.3V | 模块亦兼容 5V（板载 LDO） |
| GND | GND | 与 MCU 共地 |
| SCL | B17 (PB17) | I2C0_SCL，PF4 |
| SDA | B18 (PB18) | I2C0_SDA，PF4 |

**PB12/PB13 不支持 I2C 硬件复用**。B0/B1 现已释放，不再连接 OLED 或红外模块。

I2C 7 位地址默认 **0x3C**；若屏不亮且 SA0 已接高，可在 `oled_hw.h` 将 `OLED_HW_I2C_ADDR` 改为 **0x3D**。

软件分层：

- `src/hardware/i2c_bus.c/h` — I2C0/I2C1 两个独立控制器的非阻塞事务管理
- `src/hardware/oled_hw.c/h` — I2C0 SSD1306 初始化/写命令/写显存
- `src/middle/oled.c/h` — 1 KB 帧缓冲、6×8/8×16 字库显示、刷新
- `src/app/oled_app.c/h` — 每 500 ms 刷新：标题、Yaw、左右 RPM、八路循迹指示

上电后若 I2C 无 ACK（模块未接或地址错误），`oled_app_process()` 自动跳过，不影响心跳/电机/IMU 等功能。

与现有外设无冲突：OLED 使用 PB17/PB18，红外使用 PA15/PA16，PB0/PB1 已释放。

## 第三方库位置

`@docs/MSPM0G3519_Library` 中的开源库已复制一份到 `src/MSPM0G3519_Library`，工程实际编译使用 `src/` 下的这份拷贝（详见 `keil/.eide/eide.yml`）。`docs/MSPM0G3519_Library` 保留作为原始参考，两者内容目前一致；后续如需升级库版本，请只更新 `src/MSPM0G3519_Library` 并同步说明，避免两份代码长期不一致。
