#ifndef IMU_HW_H_
#define IMU_HW_H_

#include "zf_common_typedef.h"

/*
 * 六轴 IMU 模块硬件层（UART1，PA8 TX / PA9 RX）。
 *
 * UART1 引脚与波特率由 SysConfig 初始化；RX 中断在 imu_hw_rx_enable() 中开启。
 * TX 命令带超时，避免模块未接时阻塞死等。
 */

#define IMU_HW_RX_FIFO_SIZE        (512)
#define IMU_HW_TX_TIMEOUT_CYCLES   (8000000u)   /* 约 100 ms @ 80 MHz */

void imu_hw_init(void);
void imu_hw_rx_enable(void);
uint8 imu_hw_read_byte(uint8 *byte);
uint8 imu_hw_write_frame(const uint8 *frame, uint8 len);
uint8 imu_hw_write_reg(uint8 addr, int16 value);

#endif