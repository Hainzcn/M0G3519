#ifndef GRAYSCALE_HW_H_
#define GRAYSCALE_HW_H_

#include "zf_common_typedef.h"

/*
 * Six-channel line sensor.  It is an I2C peripheral, not an AD0/AD1/AD2
 * multiplexer.  I2C0 is shared with the OLED; sensor reads yield while an
 * OLED transfer is active.
 *
 * Wiring: SCL=PB0, SDA=PB1, GND common, sensor supply=5 V.
 * The I2C pull-up voltage must not exceed the MCU's 3.3 V I/O level.
 */

#define GRAYSCALE_HW_CHANNELS          (6u)
#define GRAYSCALE_HW_I2C_ADDRESS       (0x5Cu)
#define GRAYSCALE_HW_STATE_REGISTER    (5u)
#define GRAYSCALE_HW_SCAN_PERIOD_MS    (5u)

void grayscale_hw_init(void);
uint8 grayscale_hw_read_states(uint8 values[GRAYSCALE_HW_CHANNELS]);
uint32 grayscale_hw_get_error_count(void);

#endif
