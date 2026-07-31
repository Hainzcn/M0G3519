#ifndef GRAYSCALE_H_
#define GRAYSCALE_H_

#include "grayscale_hw.h"

#define GRAYSCALE_CHANNELS   GRAYSCALE_HW_CHANNELS

void grayscale_init(void);
void grayscale_process(void);
const uint8 *grayscale_get_values(void);
uint8 grayscale_is_scan_ready(void);
uint8 grayscale_take_scan_ready(void);
uint32 grayscale_get_scan_sequence(void);
uint8 grayscale_get_raw_bits(void);
uint8 grayscale_is_online(void);
uint32 grayscale_get_last_update_ms(void);
uint32 grayscale_get_error_count(void);

#endif
