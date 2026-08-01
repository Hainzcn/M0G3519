#ifndef IMU_HW_H_
#define IMU_HW_H_

#include "zf_common_typedef.h"

/* ATK-MS901M: UART1, PA8 TX / PA9 RX, 115200-8-N-1. */
#define IMU_HW_RX_FIFO_SIZE        (256u)
#define IMU_HW_TX_TIMEOUT_CYCLES   (8000000u)

void imu_hw_init(void);
void imu_hw_rx_enable(void);
uint8 imu_hw_read_byte(uint8 *byte);
uint8 imu_hw_write_frame(const uint8 *frame, uint8 len);
uint32 imu_hw_get_overflow_count(void);
uint8 imu_hw_take_rx_error(void);

#endif
