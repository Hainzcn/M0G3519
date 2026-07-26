#include "encoder.h"
#include "encoder_hw.h"

static uint16 encoder_left_last_raw;
static uint16 encoder_right_last_raw;
static int32 encoder_left_total;
static int32 encoder_right_total;
static int32 encoder_left_rpm;
static int32 encoder_right_rpm;

static int32 encoder_raw_delta(uint16 now, uint16 last)
{
    return (int16)(now - last);
}

static int32 encoder_count_to_rpm(int32 delta_count, uint32 period_ms)
{
    if ((0 == period_ms) || (0 == ENCODER_COUNTS_PER_WHEEL_REV))
    {
        return 0;
    }

    return (delta_count * 60000) / ((int32)ENCODER_COUNTS_PER_WHEEL_REV * (int32)period_ms);
}

static void encoder_reset_side(uint16 raw_count, uint16 *last_raw, int32 *total, int32 *rpm)
{
    *last_raw = raw_count;
    *total    = 0;
    *rpm      = 0;
}

void encoder_init(void)
{
    encoder_hw_init();
    encoder_reset_side(encoder_hw_get_raw_count(ENCODER_HW_LEFT),
                       &encoder_left_last_raw, &encoder_left_total, &encoder_left_rpm);
    encoder_reset_side(encoder_hw_get_raw_count(ENCODER_HW_RIGHT),
                       &encoder_right_last_raw, &encoder_right_total, &encoder_right_rpm);
}

uint16 encoder_get_left_raw_count(void)
{
    return encoder_hw_get_raw_count(ENCODER_HW_LEFT);
}

uint16 encoder_get_right_raw_count(void)
{
    return encoder_hw_get_raw_count(ENCODER_HW_RIGHT);
}

int32 encoder_get_left_total_count(void)
{
    return encoder_left_total;
}

int32 encoder_get_right_total_count(void)
{
    return encoder_right_total;
}

void encoder_clear_left_count(void)
{
    encoder_hw_clear_count(ENCODER_HW_LEFT);
    encoder_reset_side(encoder_hw_get_raw_count(ENCODER_HW_LEFT),
                       &encoder_left_last_raw, &encoder_left_total, &encoder_left_rpm);
}

void encoder_clear_right_count(void)
{
    encoder_hw_clear_count(ENCODER_HW_RIGHT);
    encoder_reset_side(encoder_hw_get_raw_count(ENCODER_HW_RIGHT),
                       &encoder_right_last_raw, &encoder_right_total, &encoder_right_rpm);
}

void encoder_clear_all_count(void)
{
    encoder_clear_left_count();
    encoder_clear_right_count();
}

void encoder_update_speed(uint32 period_ms)
{
    uint16 left_now;
    uint16 right_now;
    int32 left_delta;
    int32 right_delta;

    left_now    = encoder_hw_get_raw_count(ENCODER_HW_LEFT);
    right_now   = encoder_hw_get_raw_count(ENCODER_HW_RIGHT);
    left_delta  = encoder_raw_delta(left_now, encoder_left_last_raw);
    right_delta = encoder_raw_delta(right_now, encoder_right_last_raw);

    encoder_left_last_raw  = left_now;
    encoder_right_last_raw = right_now;
    encoder_left_total    += left_delta;
    encoder_right_total   += right_delta;

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
