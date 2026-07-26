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

## TB6612FNG 电机驱动（双 PWM 模式，当前采用）

| 信号 | MCU 引脚 | 库内 PWM 定义 | 说明 |
| --- | --- | --- | --- |
| 左电机 AIN1 | A0 | `PWM_TIM_A0_CH0_A0` | TIM_A0 CH0，17 kHz |
| 左电机 AIN2 | A1 | `PWM_TIM_A0_CH1_A1` | TIM_A0 CH1 |
| 右电机 BIN1 | B12 | `PWM_TIM_A0_CH2_B12` | TIM_A0 CH2 |
| 右电机 BIN2 | B13 | `PWM_TIM_A0_CH3_B13` | TIM_A0 CH3 |
| STBY / VCC | **3.3V** | - | 逻辑电源，不可接 5V |
| VM | 5~12V | - | 电机主电源 |
| GND | GND | - | 共地 |

控制逻辑（`src/hardware/motor_hw.c`）：
- **正转**：IN1=PWM，IN2=低（H,L → 正转）。
- **反转**：IN1=低，IN2=PWM（L,H → 反转）。
- **停止**：两路皆低；**刹车**：两路满占空比（H,H）。

### 双 PWM vs 符号-幅值（GPIO 方向 + 单 PWM）

| | **双 PWM（当前）** | 符号-幅值（曾用，已弃） |
| --- | --- | --- |
| 接法 | 正转：IN1=PWM，IN2=0 | 正转：IN1=1，IN2=PWM |
| TB6612 真值表 | PWM 高→H,L 驱动；低→L,L 滑行 ✓ | PWM 高→H,H **刹车**；低→H,L 驱动 ✗ 占空比反相 |
| 占空比语义 | 越大越快 ✓ | 越大越慢（除非改接法） |
| 调试 | 两路均有 PWM 纹波 | IN1 稳态 3.3V，便于万用表 |
| 定时器 | TIM_A0 四路 CH | 两路 PWM + 两路 GPIO |

**结论**：TB6612 应使用 **双 PWM**；符号-幅值仅当 PWM 接在 IN1、IN2 作方向时等价，本工程已回退双 PWM。库内 `CCCTL_23` 笔误已修复，四路可正常工作。

### 联调与故障排查

| 测量点 | 正转、占空比 70% 时预期 |
| --- | --- |
| 驱动侧 IN | 一路 PWM 平均 ~2.3V，另一路 ~0V |
| AO1–AO2 | VM 已接时有电压；恒 0 查 VM/芯片 |

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

## 第三方库位置

`@docs/MSPM0G3519_Library` 中的开源库已复制一份到 `src/MSPM0G3519_Library`，工程实际编译使用 `src/` 下的这份拷贝（详见 `keil/.eide/eide.yml`）。`docs/MSPM0G3519_Library` 保留作为原始参考，两者内容目前一致；后续如需升级库版本，请只更新 `src/MSPM0G3519_Library` 并同步说明，避免两份代码长期不一致。
