#ifndef HEARTBEAT_HW_H_
#define HEARTBEAT_HW_H_

#include "zf_common_typedef.h"

/*
 * 程序状态指示灯与心跳串口硬件层。
 *   状态灯   -> A14 (核心板板载 LED，推挽输出)
 *   心跳串口 -> UART0，TX A10 / RX A11，波特率 115200
 *   节拍来源 -> Cortex-M0+ SysTick 1 ms 中断（不占用电机 PWM 的 TIM_A0）
 *
 * UART0 由 SysConfig 生成并在 clock_init()->SYSCFG_DL_init() 中初始化；
 * 本层仅调用 DriverLib 阻塞发送，不再使用 zf_driver_uart（其波特率算法与 SysConfig 不一致）。
 */

void heartbeat_hw_init(uint32 tick_period_ms);
void heartbeat_hw_led_toggle(void);
void heartbeat_hw_uart_send_string(const char *str);
uint8 heartbeat_hw_take_tick(void);    // 有待处理周期时返回 1，并原子地取走一个周期
uint32 heartbeat_hw_get_ms(void);      // 自 SysTick 启动以来的毫秒计数，供非阻塞定时使用
uint32 heartbeat_hw_get_sequence(void); // 心跳周期计数，在 SysTick ISR 中递增

#endif
