#include "encoder_hw.h"

#include "zf_driver_encoder.h"

// 左轮：TIMG8 QEI，A 相 B10 / B 相 B11（库内示例引脚）
#define ENCODER_HW_LEFT_TIMER          (TIM_G8)
#define ENCODER_HW_LEFT_CH1            (TIMG8_ENCODER1_CH1_B10)
#define ENCODER_HW_LEFT_CH2            (TIMG8_ENCODER1_CH2_B11)

// 右轮：TIMG9 QEI。A26/A27 仅复用到 TIMG8，与左轮不能共用同一定时器，故右轮改用 TIMG9 可用引脚 B7/B9。
#define ENCODER_HW_RIGHT_TIMER         (TIM_G9)
#define ENCODER_HW_RIGHT_CH1           (TIMG9_ENCODER1_CH1_B7)
#define ENCODER_HW_RIGHT_CH2           (TIMG9_ENCODER1_CH2_B9)

static timer_index_enum encoder_hw_timer_index[ENCODER_HW_NUM] =
{
    [ENCODER_HW_LEFT]  = ENCODER_HW_LEFT_TIMER,
    [ENCODER_HW_RIGHT] = ENCODER_HW_RIGHT_TIMER,
};

void encoder_hw_init(void)
{
    encoder_quad_init(ENCODER_HW_LEFT_TIMER, ENCODER_HW_LEFT_CH1, ENCODER_HW_LEFT_CH2);
    encoder_quad_init(ENCODER_HW_RIGHT_TIMER, ENCODER_HW_RIGHT_CH1, ENCODER_HW_RIGHT_CH2);
}

uint16 encoder_hw_get_raw_count(encoder_hw_index_enum encoder)
{
    if (ENCODER_HW_NUM <= encoder)
    {
        return 0;
    }

    /* 库函数返回 int16，但位型与 16 位硬件计数一致；转为 uint16 恢复 0~65535 原始值。 */
    return (uint16)encoder_get_count(encoder_hw_timer_index[encoder]);
}

void encoder_hw_clear_count(encoder_hw_index_enum encoder)
{
    if (ENCODER_HW_NUM <= encoder)
    {
        return;
    }

    encoder_clear_count(encoder_hw_timer_index[encoder]);
}
