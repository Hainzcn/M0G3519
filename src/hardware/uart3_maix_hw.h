#ifndef UART3_MAIX_HW_H_
#define UART3_MAIX_HW_H_

#include "zf_common_typedef.h"

/* Dedicated UART3: TX PB12, RX PB13, 115200-8-N-1. */
#define UART3_MAIX_HW_RX_BUFFER_SIZE     (256u)
#define UART3_MAIX_HW_TX_BUFFER_SIZE     (256u)

void uart3_maix_hw_init(void);
uint8 uart3_maix_hw_read_byte(uint8 *data);
uint16 uart3_maix_hw_write(const uint8 *data, uint16 length);
uint16 uart3_maix_hw_write_atomic(const uint8 *data, uint16 length);
void uart3_maix_hw_send_string(const char *str);
void uart3_maix_hw_tx_pump(void);

uint32 uart3_maix_hw_get_rx_count(void);
uint32 uart3_maix_hw_get_rx_overflow_count(void);
uint32 uart3_maix_hw_get_tx_drop_count(void);

#endif
