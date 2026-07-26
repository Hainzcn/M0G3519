#ifndef ENCODER_HW_H_
#define ENCODER_HW_H_

#include "zf_common_typedef.h"

/*
 * 双路正交编码器硬件层（QEI）。
 *
 * 电机型号 GM37-520，11 线增量式编码器，减速比 30:1；线数/减速比常量见 encoder.h。
 * 逐飞库 encoder_quad_init() 仅支持 TIM_G8、TIM_G9 两路 QEI，与 TIM_A0 PWM 无冲突。
 *
 * 正交（AB 相）模式只需 2 根信号线 + 编码器电源/地；第三根引脚（如 B27）
 * 常见于 Index(Z) 相，本工程 QEI 不接入。
 */

typedef enum
{
    ENCODER_HW_LEFT = 0,
    ENCODER_HW_RIGHT,
    ENCODER_HW_NUM,
} encoder_hw_index_enum;

void encoder_hw_init(void);
int16 encoder_hw_get_count(encoder_hw_index_enum encoder);
void encoder_hw_clear_count(encoder_hw_index_enum encoder);

#endif
