#ifndef GRAYSCALE_HW_H_
#define GRAYSCALE_HW_H_

#include "zf_common_typedef.h"

/*
 * 八路循迹模块硬件层（3 位地址 + 1 位数字 OUT）。
 *
 * 当前接线（避开 A14 状态灯、异常的 PA17 和 PA18 启动配置脚）：
 *   AD0=A15, AD1=A16, AD2=A12, OUT=A13
 *
 * PA18 is the active-low BSL invoke pin sampled during BOOTRST. It must not be
 * driven by the sensor, otherwise a low OUT level can boot the MCU into ROM BSL.
 *
 * 本层仅做 GPIO 操作，不含任何延时。
 */

#define GRAYSCALE_HW_CHANNELS      (8)
#define GRAYSCALE_HW_AD0_PIN       (A15)
#define GRAYSCALE_HW_AD1_PIN       (A16)
#define GRAYSCALE_HW_AD2_PIN       (A12)
#define GRAYSCALE_HW_OUT_PIN       (A13)
#define GRAYSCALE_HW_SETTLE_US     (50)   // 供 middle 层轮询阈值

void grayscale_hw_init(void);
void grayscale_hw_select_channel(uint8 ch);
uint8 grayscale_hw_read_out(void);

#endif
