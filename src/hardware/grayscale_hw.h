#ifndef GRAYSCALE_HW_H_
#define GRAYSCALE_HW_H_

#include "zf_common_typedef.h"

/* Eight-channel infrared tracking module on dedicated I2C1 at 400 kHz. */
#define GRAYSCALE_HW_CHANNELS          (8u)
#define GRAYSCALE_HW_I2C_ADDR          (0x12u)
#define GRAYSCALE_HW_DATA_REGISTER     (0x30u)

void grayscale_hw_init(void);
void grayscale_hw_process(void);
uint8 grayscale_hw_start_read(void);
/* Returns 0 while pending, 1 on success, and 2 after a bus error. */
uint8 grayscale_hw_take_read(uint8 *sensor_bits);
uint32 grayscale_hw_get_error_count(void);

#endif
