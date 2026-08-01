#include "line_control.h"

#include "control_config.h"

/*
 * Index bit 0 is X1 (left) and bit 5 is X6 (right).  The values are
 * turn levels: negative turns left, positive turns right.  All wide-line
 * patterns (five or six active sensors) intentionally map to straight.
 */
static const int8 line_turn_lookup[1u << GRAYSCALE_CHANNELS] =
{
     0, -4, -2, -3, -1, -2, -2, -2,
     1, -2, -1, -2,  0, -1, -1, -2,
     2, -1,  0, -1,  1, -1,  0, -1,
     2,  0,  0, -1,  1,  0,  0,  0,
     4,  0,  1, -1,  2,  0,  0, -1,
     2,  0,  1,  0,  1,  0,  0,  0,
     3,  1,  1,  0,  2,  0,  1,  0,
     2,  1,  1,  0,  2,  0,  0,  0,
};

static line_control_output_t line_output;
static float line_base_rpm;
static int8 line_last_turn_level;
static uint32 line_last_valid_ms;
static uint8 line_has_valid;

static float line_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float line_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static float line_slew(float current, float target, float max_delta)
{
    if (target > (current + max_delta))
    {
        return current + max_delta;
    }
    if (target < (current - max_delta))
    {
        return current - max_delta;
    }
    return target;
}

void line_control_init(void)
{
    line_base_rpm = LINE_BASE_RPM_DEFAULT;
    line_control_reset();
}

static uint8 line_state_is_contiguous(uint8 state)
{
    uint8 index;
    uint8 seen_line = 0u;
    uint8 seen_background_after_line = 0u;

    for (index = 0u; index < GRAYSCALE_CHANNELS; index++)
    {
        if (0u != (state & (uint8)(1u << index)))
        {
            if (0u != seen_background_after_line)
            {
                return 0u;
            }
            seen_line = 1u;
        }
        else if (0u != seen_line)
        {
            seen_background_after_line = 1u;
        }
    }
    return 1u;
}

void line_control_reset(void)
{
    line_output.left_rpm = 0.0f;
    line_output.right_rpm = 0.0f;
    line_output.error = 0.0f;
    line_output.turn_rpm = 0.0f;
    line_output.active_count = 0u;
    line_output.line_valid = 0u;
    line_output.marker_detected = 0u;
    line_output.line_lost = 1u;
    line_last_turn_level = 0;
    line_last_valid_ms = 0u;
    line_has_valid = 0u;
}

void line_control_set_base_rpm(float base_rpm)
{
    line_base_rpm = line_clamp(base_rpm, 0.0f, WHEEL_TARGET_RPM_LIMIT);
}

void line_control_update(const uint8 values[GRAYSCALE_CHANNELS],
                         uint32 now_ms, float dt_s)
{
    uint8 index;
    uint8 active_count = 0u;
    uint8 state = 0u;
    uint8 state_is_contiguous;
    int8 turn_level;
    float base_rpm;
    float turn_rpm;
    float turn_limit;
    float desired_left_rpm;
    float desired_right_rpm;
    float max_target_delta;

    if (NULL == values)
    {
        return;
    }

    for (index = 0u; index < GRAYSCALE_CHANNELS; index++)
    {
        if (LINE_SENSOR_ACTIVE_LEVEL == values[index])
        {
            state |= (uint8)(1u << index);
            active_count++;
        }
    }

    state_is_contiguous = line_state_is_contiguous(state);
    line_output.active_count = active_count;
    line_output.marker_detected =
        ((0u != state_is_contiguous) &&
         (active_count >= LINE_SENSOR_MARKER_MIN_COUNT)) ? 1u : 0u;

    if ((0u != active_count) && (0u != state_is_contiguous))
    {
        if (0u != line_output.marker_detected)
        {
            turn_level = 0;
        }
        else
        {
            turn_level = line_turn_lookup[state];
        }
        line_last_turn_level = turn_level;
        line_last_valid_ms = now_ms;
        line_has_valid = 1u;
        line_output.line_valid = 1u;
        line_output.line_lost = 0u;
    }
    else if ((0u != line_has_valid) &&
             ((now_ms - line_last_valid_ms) <= LINE_LOST_HOLD_MS))
    {
        turn_level = line_last_turn_level;
        line_output.line_valid = 0u;
        line_output.line_lost = 0u;
    }
    else
    {
        line_output.left_rpm = 0.0f;
        line_output.right_rpm = 0.0f;
        line_output.error = (float)line_last_turn_level;
        line_output.turn_rpm = 0.0f;
        line_output.line_valid = 0u;
        line_output.line_lost = 1u;
        return;
    }

    base_rpm = line_base_rpm - (line_abs((float)turn_level) *
        LINE_TABLE_CURVE_RPM_REDUCTION);
    base_rpm = line_clamp(base_rpm, LINE_MIN_RPM_DEFAULT,
                          WHEEL_TARGET_RPM_LIMIT);
    turn_rpm = (float)turn_level * LINE_TABLE_TURN_RPM_STEP;
    turn_limit = base_rpm - LINE_TABLE_MIN_INNER_RPM;
    turn_rpm = line_clamp(turn_rpm, -turn_limit, turn_limit);

    line_output.error = (float)turn_level;
    line_output.turn_rpm = turn_rpm;
    desired_left_rpm = line_clamp(base_rpm + turn_rpm,
        -WHEEL_TARGET_RPM_LIMIT, WHEEL_TARGET_RPM_LIMIT);
    desired_right_rpm = line_clamp(base_rpm - turn_rpm,
        -WHEEL_TARGET_RPM_LIMIT, WHEEL_TARGET_RPM_LIMIT);
    max_target_delta = LINE_TARGET_SLEW_RPM_PER_S * dt_s;
    line_output.left_rpm = line_slew(line_output.left_rpm,
        desired_left_rpm, max_target_delta);
    line_output.right_rpm = line_slew(line_output.right_rpm,
        desired_right_rpm, max_target_delta);
}

const line_control_output_t *line_control_get_output(void)
{
    return &line_output;
}
