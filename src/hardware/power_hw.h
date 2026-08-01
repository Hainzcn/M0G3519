#ifndef POWER_HW_H_
#define POWER_HW_H_

#include "zf_common_typedef.h"

void power_hw_init(void);
uint8 power_hw_read_raw(uint16 *raw);
uint32 power_hw_get_error_count(void);

#endif
