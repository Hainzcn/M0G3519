#ifndef BUTTON_HW_H_
#define BUTTON_HW_H_

#include "zf_common_typedef.h"

/*
 * 用户按键硬件层（逐飞核心板排针）：
 *   A30 -> SW1
 *   A31 -> SW2
 *   B1  -> SW3
 *   B0  -> SW4
 *
 * 按键一端接 MCU 引脚，另一端接 GND；MCU 内部上拉，按下为低电平。
 */

#define BUTTON_HW_COUNT             (4)

#define BUTTON_HW_ACTIVE_LEVEL      (0)

void  button_hw_init(void);
uint8 button_hw_read_raw(uint8 index);

#endif
