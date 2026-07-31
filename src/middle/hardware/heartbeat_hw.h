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
 * TX 使用环形缓冲非阻塞发送，由主循环按硬件 FIFO 空位批量泵出。
 */

#define HEARTBEAT_HW_TX_FIFO_SIZE      (1024)

void heartbeat_hw_init(uint32 tick_period_ms);
void heartbeat_hw_led_toggle(void);
void heartbeat_hw_uart_send_string(const char *str);
void heartbeat_hw_uart_tx_pump(void);
void heartbeat_hw_uart_flush_blocking(void);
uint8 heartbeat_hw_uart_tx_busy(void);
uint32 heartbeat_hw_uart_get_drop_count(void);
uint8 heartbeat_hw_take_tick(void);
uint32 heartbeat_hw_get_ms(void);
uint32 heartbeat_hw_get_sequence(void);

#endif
