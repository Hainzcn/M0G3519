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

## TB6612FNG 电机驱动（已实现，开环双 PWM 模式）

| 信号 | MCU 引脚 | 库内 PWM 定义 | 说明 |
| --- | --- | --- | --- |
| 左电机 AIN1 | A0 | `PWM_TIM_A0_CH0_A0` | TIM_A0 CH0，17 kHz |
| 左电机 AIN2 | A1 | `PWM_TIM_A0_CH1_A1` | TIM_A0 CH1，17 kHz |
| 右电机 BIN1 | B12 | `PWM_TIM_A0_CH2_B12` | TIM_A0 CH2，17 kHz |
| 右电机 BIN2 | B13 | `PWM_TIM_A0_CH3_B13` | TIM_A0 CH3，17 kHz |
| STBY | - | - | 已由驱动板硬件电路直接拉高，MCU 不控制 |
| 电源 | VM / GND | - | 电机电源与逻辑 3V3 分离，与 MCU 共地 |

四路 PWM 全部落在 `TIM_A0` 的 CH0~CH3，共用同一频率寄存器，占空比寄存器各自独立。控制逻辑（双 PWM 驱动，见 `src/hardware/motor_hw.c`）：
- 正转：对应 IN1 输出 PWM，IN2 输出低电平。
- 反转：对应 IN2 输出 PWM，IN1 输出低电平。
- 占空比为 0：两路同为低电平（滑行停止）；`motor_hw_brake()` 会将两路同时拉满（短路刹车）。

## 编码器（预留占位，尚未初始化）

| 侧别 | 占位引脚 | 说明 |
| --- | --- | --- |
| 左轮 | B10 / B11 / B9 | 待确认具体 A 相 / B 相 / 备用信号分配后再调用 `encoder_quad_init` |
| 右轮 | A26 / B27 / A27 | 同上，右轮 |

当前阶段电机为开环驱动，代码中未调用 `encoder_quad_init` / `encoder_dir_init`，避免引入未确认的外设配置。后续如需速度闭环，请先核对 `src/MSPM0G3519_Library/zf_driver/zf_driver_encoder.h` 内的引脚枚举与上表实际接线的对应关系，再在 `src/middle/motor.c` 中新增闭环入口（预留位置见该文件注释）。

## 程序状态指示灯与心跳串口（已实现）

| 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| 状态灯 | A14 | 核心板板载 LED，推挽输出；上电默认熄灭，每 500ms 翻转一次 |
| UART0 TX | A10 | 心跳报文发送，115200-8-N-1，由 SysConfig 初始化 |
| UART0 RX | A11 | 预留（当前心跳功能不接收数据） |

串口接线说明（MCU 侧均为 UART0，引脚固定为 PA10/PA11）：

| 连接方式 | 说明 |
| --- | --- |
| 核心板排针 A10/A11 | PA10 为 MCU 发送端，需接 USB-TTL 的 **RX**；PA11 接 USB-TTL 的 **TX**；必须共地 |
| 板底 SWD 排针 RX/TX | 这是 XDS110 下载器转出的 UART0，**不是** PA13/PA14；下载器内部已接到 PA10/PA11。使用 XDS110 时请打开设备管理器里带 **Application/User UART** 字样的 COM 口 |
| IO 底板 USB-C（若有 CH340） | 同样复用 UART0（PA10/PA11），波特率请设为 **115200**（本工程固定 115200，不是部分例程默认的 9600） |

上电后应先收到 `BOOT OK`，之后每 500ms 收到 `HEARTBEAT,<计数值>`。若 LED 正常闪烁但串口仍无数据，优先检查 COM 口选择与波特率，其次确认 TX/RX 是否交叉、是否共地。

节拍来源使用 Cortex-M0+ 的 `SysTick` 1ms 中断，不占用电机 PWM 使用的 `TIM_A0` 或其他通用定时器。中断中只累积待处理标志；主循环调用 `heartbeat_app_process()` 后才翻转状态灯并通过 UART0 发送心跳报文。`main()` 必须通过 `clock_init()` 完成外设上电，否则 UART0 无法工作。

## 第三方库位置

`@docs/MSPM0G3519_Library` 中的开源库已复制一份到 `src/MSPM0G3519_Library`，工程实际编译使用 `src/` 下的这份拷贝（详见 `keil/.eide/eide.yml`）。`docs/MSPM0G3519_Library` 保留作为原始参考，两者内容目前一致；后续如需升级库版本，请只更新 `src/MSPM0G3519_Library` 并同步说明，避免两份代码长期不一致。
