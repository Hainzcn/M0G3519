#ifndef POWER_H_
#define POWER_H_

#include "zf_common_typedef.h"

void power_init(void);
void power_process(void);
uint16 power_get_bus_millivolts(void);
uint16 power_get_raw(void);
uint8 power_is_valid(void);

#endif
