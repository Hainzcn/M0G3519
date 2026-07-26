#include "encoder.h"
#include "encoder_hw.h"

static int16 encoder_left_last_count;
static int16 encoder_right_last_count;
static int32 encoder_left_rpm;
static int32 encoder_right_rpm;

static int32 encoder_count_to_rpm(int32 delta_count, uint32 period_ms)
{
    if ((0 == period_ms) || (0 == ENCODER_COUNTS_PER_WHEEL_REV))
    {
        return 0;
    }

    return (delta_count * 60000) / ((int32)ENCODER_COUNTS_PER_WHEEL_REV * (int32)period_ms);
}

static void encoder_reset_speed_state(int16 left_count, int16 right_count)
{
    encoder_left_last_count  = left_count;
    encoder_right_last_count = right_count;
    encoder_left_rpm         = 0;
    encoder_right_rpm        = 0;
}

void encoder_init(void)
{
    encoder_hw_init();
    encoder_reset_speed_state(encoder_hw_get_count(ENCODER_HW_LEFT),
                              encoder_hw_get_count(ENCODER_HW_RIGHT));
}

int16 encoder_get_left_count(void)
{
    return encoder_hw_get_count(ENCODER_HW_LEFT);
}

int16 encoder_get_right_count(void)
{
    return encoder_hw_get_count(ENCODER_HW_RIGHT);
}

void encoder_clear_left_count(void)
{
    encoder_hw_clear_count(ENCODER_HW_LEFT);
    encoder_left_last_count = encoder_hw_get_count(ENCODER_HW_LEFT);
    encoder_left_rpm        = 0;
}

void encoder_clear_right_count(void)
{
    encoder_hw_clear_count(ENCODER_HW_RIGHT);
    encoder_right_last_count = encoder_hw_get_count(ENCODER_HW_RIGHT);
    encoder_right_rpm        = 0;
}

void encoder_update_speed(uint32 period_ms)
{
    int16 left_now;
    int16 right_now;
    int32 left_delta;
    int32 right_delta;

    left_now   = encoder_hw_get_count(ENCODER_HW_LEFT);
    right_now  = encoder_hw_get_count(ENCODER_HW_RIGHT);
    left_delta = (int32)left_now - (int32)encoder_left_last_count;
    right_delta = (int32)right_now - (int32)encoder_right_last_count;

    encoder_left_last_count  = left_now;
    encoder_right_last_count = right_now;

    encoder_left_rpm  = encoder_count_to_rpm(left_delta, period_ms);
    encoder_right_rpm = encoder_count_to_rpm(right_delta, period_ms);
}

int32 encoder_get_left_rpm(void)
{
    return encoder_left_rpm;
}

int32 encoder_get_right_rpm(void)
{
    return encoder_right_rpm;
}
