#ifndef IMU_HW_H_
#define IMU_HW_H_

#include "zf_common_typedef.h"

/*
 * ATK-MS901M hardware layer: UART1, PA8 TX / PA9 RX, 115200-8-N-1.
 *
 * UART RX is moved into small fixed blocks by DMA. At 115200 baud a 16-byte
 * block adds at most about 1.4 ms before parsing; parsing stays in main loop.
 */

#define IMU_HW_DMA_BLOCK_SIZE      (16u)
#define IMU_HW_DMA_BLOCK_COUNT     (8u)
#define IMU_HW_TX_TIMEOUT_CYCLES   (8000000u)

void imu_hw_init(void);
void imu_hw_rx_enable(void);
const uint8 *imu_hw_acquire_block(uint16 *length);
void imu_hw_release_block(const uint8 *block);
uint8 imu_hw_write_frame(const uint8 *frame, uint8 len);
uint32 imu_hw_get_overflow_count(void);
uint32 imu_hw_get_dma_block_count(void);
uint8 imu_hw_take_rx_error(void);

#endif
