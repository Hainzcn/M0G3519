#ifndef BUZZER_HW_H_
#define BUZZER_HW_H_

#include "zf_common_typedef.h"

/*
 * Active buzzer control:
 *   PA13 -> transistor/module signal input
 *
 * The default circuit is active high. Set BUZZER_HW_ACTIVE_LEVEL to 0u for
 * a low-level-triggered buzzer module.
 */
#define BUZZER_HW_ACTIVE_LEVEL    (1u)

void buzzer_hw_init(void);
void buzzer_hw_set_enabled(uint8 enabled);

#endif
