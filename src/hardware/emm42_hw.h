#ifndef EMM42_HW_H_
#define EMM42_HW_H_

#include "zf_common_typedef.h"

/* Dedicated UART7: TX PA23, RX PA24, 115200 8N1. */
#define EMM42_HW_RX_BUFFER_SIZE       (128u)
#define EMM42_HW_TX_TIMEOUT_LOOPS     (100000u)

void emm42_hw_init(void);
uint8 emm42_hw_write(const uint8 *data, uint16 length);
uint8 emm42_hw_read_byte(uint8 *data);
void emm42_hw_clear_rx(void);
uint32 emm42_hw_get_rx_overflow_count(void);

#endif
