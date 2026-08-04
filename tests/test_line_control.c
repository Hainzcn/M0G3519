#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "encoder.h"
#include "line_control.h"

static int32 mock_left_count;
static int32 mock_right_count;

int32 encoder_get_left_total_count(void) { return mock_left_count; }
int32 encoder_get_right_total_count(void) { return mock_right_count; }

static float center_rpm(const line_control_output_t *output)
{
    return 0.5f * (output->left_rpm + output->right_rpm);
}

static void test_base_speed_uses_jerk_limited_envelope(void)
{
    const float dt_s = 0.01f;
    const uint8 centered_values[GRAYSCALE_CHANNELS] =
        {0u, 0u, 0u, 1u, 1u, 0u, 0u, 0u};
    const line_control_output_t *output;
    float previous_rpm = 0.0f;
    float previous_accel_rpm_s = 0.0f;
    float peak_accel_rpm_s = 0.0f;
    float accel_rpm_s;
    float jerk_rpm_s2;
    uint32 index;
    uint8 acceleration_rolled_off = 0u;

    line_control_init();
    line_control_set_base_rpm(TRACK_MODE_3_LINE_FOLLOW_RPM);
    output = line_control_get_output();

    for (index = 0u; index < 100u; index++)
    {
        line_control_update(centered_values, index * 10u, dt_s);
        accel_rpm_s = (center_rpm(output) - previous_rpm) / dt_s;
        jerk_rpm_s2 = (accel_rpm_s - previous_accel_rpm_s) / dt_s;

        assert(center_rpm(output) >= previous_rpm - 0.001f);
        assert(center_rpm(output) <= TRACK_MODE_3_LINE_FOLLOW_RPM + 0.001f);
        assert(accel_rpm_s <=
               LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S + 0.1f);
        assert(fabsf(jerk_rpm_s2) <=
               LINE_LOOKUP_BASE_JERK_RPM_PER_S2 + 1.0f);
        if (accel_rpm_s > peak_accel_rpm_s)
        {
            peak_accel_rpm_s = accel_rpm_s;
        }
        else if ((peak_accel_rpm_s > 50.0f) &&
                 (accel_rpm_s < peak_accel_rpm_s - 1.0f))
        {
            acceleration_rolled_off = 1u;
        }
        previous_rpm = center_rpm(output);
        previous_accel_rpm_s = accel_rpm_s;
    }

    assert(center_rpm(output) == TRACK_MODE_3_LINE_FOLLOW_RPM);
    assert(peak_accel_rpm_s > 100.0f);
    assert(0u != acceleration_rolled_off);
}

static void test_zero_target_is_not_clamped_to_minimum_speed(void)
{
    line_control_init();
    line_control_set_base_rpm(20.0f);
    assert(line_control_get_base_rpm() == 20.0f);
    line_control_set_base_rpm(0.0f);
    assert(line_control_get_base_rpm() == 0.0f);
}

static void test_base_acceleration_preview_does_not_advance_live_state(void)
{
    const float preview_s = BALANCE_SIMPLE_CAR_FF_PREVIEW_S;
    float current_accel;
    float preview_accel;

    line_control_init();
    line_control_set_base_rpm(TRACK_MODE_3_LINE_FOLLOW_RPM);
    current_accel = line_control_get_base_accel_rpm_s();
    preview_accel = line_control_get_base_accel_preview_rpm_s(preview_s);

    assert(current_accel == 0.0f);
    assert(line_control_get_base_accel_rpm_s() == current_accel);
    assert(preview_accel > current_accel);
    assert(preview_accel <= LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S);
    assert(fabsf(preview_accel -
        LINE_LOOKUP_BASE_JERK_RPM_PER_S2 * preview_s) < 0.1f);

    line_control_set_base_rpm_immediate(TRACK_MODE_4_LINE_FOLLOW_RPM);
    line_control_set_base_rpm(0.0f);
    current_accel = line_control_get_base_accel_rpm_s();
    preview_accel = line_control_get_base_accel_preview_rpm_s(preview_s);
    assert(current_accel == 0.0f);
    assert(line_control_get_base_accel_rpm_s() == current_accel);
    assert(preview_accel < 0.0f);
}

static void test_wide_letter_pattern_holds_last_valid_direction(void)
{
    const float dt_s = 0.01f;
    const uint8 left_line[GRAYSCALE_CHANNELS] =
        {1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    const uint8 letter_pattern[GRAYSCALE_CHANNELS] =
        {0u, 0u, 0u, 1u, 1u, 1u, 1u, 1u};
    const uint8 centered_line[GRAYSCALE_CHANNELS] =
        {0u, 0u, 0u, 1u, 1u, 0u, 0u, 0u};
    const line_control_output_t *output;
    uint32 index;
    int8 held_level;

    line_control_init();
    line_control_set_base_rpm(NO_LOAD_LAP_CRUISE_RPM);
    output = line_control_get_output();

    for (index = 0u; index < 5u; index++)
    {
        line_control_update(left_line, index * 10u, dt_s);
    }
    held_level = output->lookup_level;
    assert(held_level < 0);

    for (index = 5u; index < 30u; index++)
    {
        line_control_update(letter_pattern, index * 10u, dt_s);
        assert(output->active_count == 5u);
        assert(0u != output->wide_pattern_filtered);
        assert(0u == output->line_valid);
        assert(0u == output->line_lost);
        assert(output->lookup_level == held_level);
    }

    line_control_update(centered_line, 300u, dt_s);
    assert(0u == output->wide_pattern_filtered);
    assert(0u != output->line_valid);
    assert(0u == output->line_lost);
}

int main(void)
{
    test_base_speed_uses_jerk_limited_envelope();
    test_zero_target_is_not_clamped_to_minimum_speed();
    test_base_acceleration_preview_does_not_advance_live_state();
    test_wide_letter_pattern_holds_last_valid_direction();
    puts("line control acceleration envelope tests passed");
    return 0;
}
