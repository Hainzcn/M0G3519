#include "line_control.h"

#include "control_config.h"
#include "encoder.h"

#define LINE_PI_F                         (3.14159265f)
#define LINE_LEVEL_HISTORY_SIZE           (3u)

typedef struct
{
    uint8 mask;
    int8 level;
} line_pattern_entry_t;

/* Bit i is one when grayscale channel i sees black. */
static const line_pattern_entry_t line_pattern_table[] =
{
    {0x01u, -4}, {0x02u, -3}, {0x04u, -2}, {0x08u, -1},
    {0x10u,  1}, {0x20u,  2}, {0x40u,  3}, {0x80u,  4},
    {0x03u, -4}, {0x06u, -3}, {0x0Cu, -2}, {0x18u,  0},
    {0x30u,  2}, {0x60u,  3}, {0xC0u,  4},
    {0x07u, -3}, {0x0Eu, -2}, {0x1Cu, -1}, {0x38u,  1},
    {0x70u,  2}, {0xE0u,  3},
    {0x0Fu, -2}, {0x1Eu, -1}, {0x3Cu,  0}, {0x78u,  1},
    {0xF0u,  2},
    {0x1Fu, -2}, {0x3Eu, -1}, {0x7Cu,  1}, {0xF8u,  2},
};

static line_control_output_t line_output;
static float line_straight_base_rpm;
static int8 line_level_history[LINE_LEVEL_HISTORY_SIZE];
static uint8 line_level_filter_initialized;
static int8 line_last_level;
static uint8 line_has_valid;
static uint8 line_invalid_active;
static uint32 line_invalid_start_ms;
static int32 line_last_left_count;
static int32 line_last_right_count;
static uint32 line_phase_transition_count;
static int8 line_command_level;
static uint8 line_command_level_initialized;
static uint8 line_reverse_hold_active;
static uint32 line_reverse_hold_start_ms;
static float line_applied_base_rpm;
static float line_applied_base_accel_rpm_s;
static float line_applied_turn_rpm;

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

static void line_apply_base_envelope(float target_rpm, float dt_s)
{
    float error_rpm = target_rpm - line_applied_base_rpm;
    float direction = (error_rpm >= 0.0f) ? 1.0f : -1.0f;
    float directed_accel_rpm_s =
        line_applied_base_accel_rpm_s * direction;
    float stopping_delta_rpm = 0.0f;
    float sample_lookahead_rpm = 0.0f;
    float target_accel_rpm_s;
    float next_rpm;

    if (directed_accel_rpm_s > 0.0f)
    {
        stopping_delta_rpm =
            directed_accel_rpm_s * directed_accel_rpm_s /
            (2.0f * LINE_LOOKUP_BASE_JERK_RPM_PER_S2);
        /* Start roll-off early enough for the two discrete boundary samples. */
        sample_lookahead_rpm = 2.0f * directed_accel_rpm_s * dt_s;
    }
    target_accel_rpm_s =
        ((direction * error_rpm) <=
         (stopping_delta_rpm + sample_lookahead_rpm)) ? 0.0f :
        direction * LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S;
    line_applied_base_accel_rpm_s = line_slew(
        line_applied_base_accel_rpm_s,
        target_accel_rpm_s,
        LINE_LOOKUP_BASE_JERK_RPM_PER_S2 * dt_s);

    next_rpm = line_applied_base_rpm +
        line_applied_base_accel_rpm_s * dt_s;
    if (((error_rpm >= 0.0f) && (next_rpm >= target_rpm)) ||
        ((error_rpm < 0.0f) && (next_rpm <= target_rpm)))
    {
        line_applied_base_rpm = target_rpm;
        line_applied_base_accel_rpm_s = 0.0f;
    }
    else
    {
        line_applied_base_rpm = next_rpm;
    }
}

static uint8 line_abs_level(int8 level)
{
    return (uint8)((level < 0) ? -level : level);
}

static float line_speed_scale(void)
{
    return line_straight_base_rpm / LINE_LOOKUP_SPEED_REFERENCE_RPM;
}

static float line_feedback_scale(void)
{
    return line_clamp(line_speed_scale(), 0.0f,
        LINE_LOOKUP_FEEDBACK_SCALE_MAX);
}

static int8 line_median3(int8 a, int8 b, int8 c)
{
    if (a > b)
    {
        int8 temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c)
    {
        int8 temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b)
    {
        b = a;
    }
    return b;
}

static int8 line_filter_level(int8 level)
{
    if (0u == line_level_filter_initialized)
    {
        line_level_history[0] = level;
        line_level_history[1] = level;
        line_level_history[2] = level;
        line_level_filter_initialized = 1u;
    }
    else
    {
        line_level_history[0] = line_level_history[1];
        line_level_history[1] = line_level_history[2];
        line_level_history[2] = level;
    }

    return line_median3(line_level_history[0],
                        line_level_history[1],
                        line_level_history[2]);
}

static int8 line_shape_level(int8 level, uint32 now_ms)
{
    if (0u == line_command_level_initialized)
    {
        line_command_level = level;
        line_command_level_initialized = 1u;
        return line_command_level;
    }

    if (0u != line_reverse_hold_active)
    {
        if ((now_ms - line_reverse_hold_start_ms) <
            LINE_LOOKUP_REVERSE_HOLD_MS)
        {
            line_command_level = 0;
            return line_command_level;
        }
        line_reverse_hold_active = 0u;
    }

    if (((line_command_level < 0) && (level > 0)) ||
        ((line_command_level > 0) && (level < 0)))
    {
        line_command_level += (line_command_level < 0) ? 1 : -1;
        if (0 == line_command_level)
        {
            line_reverse_hold_active = 1u;
            line_reverse_hold_start_ms = now_ms;
        }
        return line_command_level;
    }

    if (line_command_level < level)
    {
        line_command_level++;
    }
    else if (line_command_level > level)
    {
        line_command_level--;
    }
    return line_command_level;
}

static uint8 line_lookup_level(uint8 mask, int8 *level)
{
    uint8 index;

    if (NULL == level)
    {
        return 0u;
    }

    for (index = 0u;
         index < (uint8)(sizeof(line_pattern_table) /
                         sizeof(line_pattern_table[0]));
         index++)
    {
        if (line_pattern_table[index].mask == mask)
        {
            *level = line_pattern_table[index].level;
            return 1u;
        }
    }
    return 0u;
}

static uint8 line_build_mask(const uint8 values[GRAYSCALE_CHANNELS],
                             uint8 *black_count,
                             uint8 *right_count)
{
    uint8 index;
    uint8 mask = 0u;
    uint8 black = 0u;
    uint8 right = 0u;

    for (index = 0u; index < GRAYSCALE_CHANNELS; index++)
    {
        if (LINE_BLACK_ACTIVE_LEVEL == values[index])
        {
            mask |= (uint8)(1u << index);
            black++;
            if (index >= 5u)
            {
                right++;
            }
        }
    }

    *black_count = black;
    *right_count = right;
    return mask;
}

static float line_lookup_turn_rpm(int8 level)
{
    float magnitude;
    float feedback_scale = line_feedback_scale();

    switch (line_abs_level(level))
    {
        case 1u:
            magnitude = LINE_LOOKUP_LEVEL_1_TURN_RPM;
            break;
        case 2u:
            magnitude = LINE_LOOKUP_LEVEL_2_TURN_RPM;
            break;
        case 3u:
            magnitude = LINE_LOOKUP_LEVEL_3_TURN_RPM;
            break;
        case 4u:
            magnitude = LINE_LOOKUP_LEVEL_4_TURN_RPM;
            break;
        default:
            magnitude = 0.0f;
            break;
    }

    if (level < 0)
    {
        magnitude = -magnitude;
    }
    return LINE_LOOKUP_CORRECTION_SIGN * magnitude * feedback_scale;
}

static float line_phase_length_m(line_track_phase_t phase)
{
    return (LINE_TRACK_PHASE_RIGHT_ARC == phase) ?
        LINE_LOOKUP_ARC_LENGTH_M : LINE_LOOKUP_STRAIGHT_LENGTH_M;
}

static float line_smoothstep(float value)
{
    value = line_clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

static float line_curve_blend(void)
{
    float distance_m = line_output.phase_distance_m;
    float half_length_m = LINE_LOOKUP_TRANSITION_HALF_LENGTH_M;
    float phase_length_m = line_phase_length_m(line_output.phase);
    float progress;

    if (half_length_m <= 0.0f)
    {
        return (LINE_TRACK_PHASE_RIGHT_ARC == line_output.phase) ?
            1.0f : 0.0f;
    }

    if (LINE_TRACK_PHASE_RIGHT_ARC == line_output.phase)
    {
        if (distance_m < half_length_m)
        {
            progress = line_smoothstep(distance_m / half_length_m);
            return 0.5f + 0.5f * progress;
        }
        if (distance_m > (phase_length_m - half_length_m))
        {
            progress = line_smoothstep(
                (phase_length_m - distance_m) / half_length_m);
            return 0.5f + 0.5f * progress;
        }
        return 1.0f;
    }

    if ((0u != line_phase_transition_count) &&
        (distance_m < half_length_m))
    {
        progress = line_smoothstep(distance_m / half_length_m);
        return 0.5f * (1.0f - progress);
    }
    if (distance_m > (phase_length_m - half_length_m))
    {
        progress = line_smoothstep(
            (distance_m - (phase_length_m - half_length_m)) /
            half_length_m);
        return 0.5f * progress;
    }
    return 0.0f;
}

static void line_update_phase_distance(void)
{
    int32 left_count = encoder_get_left_total_count();
    int32 right_count = encoder_get_right_total_count();
    int32 left_delta = left_count - line_last_left_count;
    int32 right_delta = right_count - line_last_right_count;
    float center_counts;
    float distance_delta_m;
    float phase_length_m;

    line_last_left_count = left_count;
    line_last_right_count = right_count;
    center_counts = 0.5f *
        ((float)left_delta * WHEEL_LEFT_ENCODER_SIGN +
         (float)right_delta * WHEEL_RIGHT_ENCODER_SIGN);
    distance_delta_m = center_counts *
        (LINE_PI_F * CHASSIS_WHEEL_DIAMETER_M) /
        (float)ENCODER_COUNTS_PER_WHEEL_REV;

    if (distance_delta_m > 0.0f)
    {
        line_output.phase_distance_m += distance_delta_m;
    }

    phase_length_m = line_phase_length_m(line_output.phase);
    while (line_output.phase_distance_m >= phase_length_m)
    {
        line_output.phase_distance_m -= phase_length_m;
        line_output.phase =
            (LINE_TRACK_PHASE_STRAIGHT == line_output.phase) ?
            LINE_TRACK_PHASE_RIGHT_ARC : LINE_TRACK_PHASE_STRAIGHT;
        line_phase_transition_count++;
        phase_length_m = line_phase_length_m(line_output.phase);
    }
    line_output.right_curve_detected =
        (LINE_TRACK_PHASE_RIGHT_ARC == line_output.phase) ? 1u : 0u;
}

static float line_phase_base_rpm(void)
{
    line_output.curve_blend = line_curve_blend();
    return line_straight_base_rpm;
}

static float line_phase_turn_rpm(float base_rpm)
{
    return line_output.curve_blend * base_rpm * CHASSIS_WHEEL_TRACK_M /
        (2.0f * LINE_LOOKUP_ARC_RADIUS_M);
}

static void line_apply_targets(float base_rpm, float phase_turn_rpm,
                               float lookup_turn_rpm, float dt_s)
{
    float feedback_scale = line_feedback_scale();
    float target_turn_rpm = line_clamp(phase_turn_rpm + lookup_turn_rpm,
        -LINE_LOOKUP_TURN_RPM_LIMIT * feedback_scale,
        LINE_LOOKUP_TURN_RPM_LIMIT * feedback_scale);
    float max_turn_delta = LINE_LOOKUP_TURN_SLEW_RPM_PER_S * dt_s;

    line_apply_base_envelope(base_rpm, dt_s);
    line_applied_turn_rpm = line_slew(line_applied_turn_rpm,
        target_turn_rpm, max_turn_delta);

    line_output.pid_turn_rpm = lookup_turn_rpm;
    line_output.lookup_correction_rpm = lookup_turn_rpm;
    line_output.curvature_feedforward_rpm = phase_turn_rpm;
    line_output.turn_rpm = line_applied_turn_rpm;
    line_output.left_rpm = line_clamp(
        line_applied_base_rpm + line_applied_turn_rpm,
        -WHEEL_TARGET_RPM_LIMIT, WHEEL_TARGET_RPM_LIMIT);
    line_output.right_rpm = line_clamp(
        line_applied_base_rpm - line_applied_turn_rpm,
        -WHEEL_TARGET_RPM_LIMIT, WHEEL_TARGET_RPM_LIMIT);
}

static void line_apply_level(int8 level, float dt_s)
{
    float base_rpm = line_phase_base_rpm();
    float lookup_turn_rpm = line_lookup_turn_rpm(level);
    float phase_turn_rpm;

    phase_turn_rpm = line_phase_turn_rpm(base_rpm);
    line_apply_targets(base_rpm, phase_turn_rpm, lookup_turn_rpm, dt_s);
    line_output.error = (float)level * 1000.0f;
    line_output.lookup_level = level;
}

void line_control_init(void)
{
    line_straight_base_rpm = LINE_LOOKUP_STRAIGHT_BASE_RPM;
    line_control_reset();
}

void line_control_reset(void)
{
    line_output.left_rpm = 0.0f;
    line_output.right_rpm = 0.0f;
    line_output.error = 0.0f;
    line_output.pid_turn_rpm = 0.0f;
    line_output.curvature_feedforward_rpm = 0.0f;
    line_output.turn_rpm = 0.0f;
    line_output.lookup_correction_rpm = 0.0f;
    line_output.phase_distance_m = 0.0f;
    line_output.curve_blend = 0.0f;
    line_output.speed_scale = line_speed_scale();
    line_output.feedback_scale = line_feedback_scale();
    line_output.active_count = 0u;
    line_output.right_active_count = 0u;
    line_output.right_curve_detected =
        (0u != LINE_LOOKUP_INITIAL_PHASE) ? 1u : 0u;
    line_output.sensor_mask = 0u;
    line_output.lookup_level = 0;
    line_output.phase = (0u != LINE_LOOKUP_INITIAL_PHASE) ?
        LINE_TRACK_PHASE_RIGHT_ARC : LINE_TRACK_PHASE_STRAIGHT;
    line_output.line_valid = 0u;
    line_output.marker_detected = 0u;
    line_output.wide_pattern_filtered = 0u;
    line_output.line_lost = 1u;
    line_level_filter_initialized = 0u;
    line_last_level = 0;
    line_command_level = 0;
    line_command_level_initialized = 0u;
    line_reverse_hold_active = 0u;
    line_reverse_hold_start_ms = 0u;
    line_applied_base_rpm = 0.0f;
    line_applied_base_accel_rpm_s = 0.0f;
    line_applied_turn_rpm = 0.0f;
    line_has_valid = 0u;
    line_invalid_active = 0u;
    line_invalid_start_ms = 0u;
    line_last_left_count = encoder_get_left_total_count();
    line_last_right_count = encoder_get_right_total_count();
    line_phase_transition_count = 0u;
}

void line_control_set_base_rpm(float base_rpm)
{
    line_straight_base_rpm = line_clamp(base_rpm,
        0.0f, LINE_LOOKUP_SPEED_MAX_RPM);
    line_output.speed_scale = line_speed_scale();
    line_output.feedback_scale = line_feedback_scale();
}

void line_control_set_base_rpm_immediate(float base_rpm)
{
    line_straight_base_rpm = line_clamp(base_rpm,
        0.0f, LINE_LOOKUP_SPEED_MAX_RPM);
    line_applied_base_rpm = line_straight_base_rpm;
    line_applied_base_accel_rpm_s = 0.0f;
    line_output.speed_scale = line_speed_scale();
    line_output.feedback_scale = line_feedback_scale();
}

float line_control_get_base_rpm(void)
{
    return line_straight_base_rpm;
}

void line_control_update(const uint8 values[GRAYSCALE_CHANNELS],
                         uint32 now_ms, float dt_s)
{
    uint8 black_count;
    uint8 right_count;
    uint8 mask;
    uint8 pattern_valid;
    int8 raw_level = 0;
    uint32 invalid_elapsed_ms;

    if ((NULL == values) || (dt_s <= 0.0f))
    {
        return;
    }

    line_update_phase_distance();
    mask = line_build_mask(values, &black_count, &right_count);
    line_output.sensor_mask = mask;
    line_output.active_count = black_count;
    line_output.right_active_count = right_count;
    line_output.marker_detected =
        (black_count >= LINE_SENSOR_MARKER_MIN_COUNT) ? 1u : 0u;
    line_output.wide_pattern_filtered =
        (black_count >= LINE_SENSOR_WIDE_PATTERN_MIN_COUNT) ? 1u : 0u;

    /* Letters and transverse markers are observations, not steering input. */
    if (0u != line_output.wide_pattern_filtered)
    {
        line_invalid_active = 0u;
        line_output.line_valid = 0u;
        line_output.line_lost = 0u;
        line_apply_level(line_last_level, dt_s);
        return;
    }

    pattern_valid = line_lookup_level(mask, &raw_level);

    if (0u != pattern_valid)
    {
        line_last_level = line_shape_level(
            line_filter_level(raw_level), now_ms);
        line_has_valid = 1u;
        line_invalid_active = 0u;
        line_output.line_valid = 1u;
        line_output.line_lost = 0u;
        line_apply_level(line_last_level, dt_s);
        return;
    }

    if (0u == line_invalid_active)
    {
        line_invalid_active = 1u;
        line_invalid_start_ms = now_ms;
    }
    invalid_elapsed_ms = now_ms - line_invalid_start_ms;
    line_output.line_valid = 0u;

    if ((0u != line_has_valid) &&
        (invalid_elapsed_ms <= LINE_LOOKUP_LOST_HOLD_MS))
    {
        line_output.line_lost = 0u;
        line_apply_level(line_last_level, dt_s);
        return;
    }

    if (invalid_elapsed_ms <= LINE_LOOKUP_SEARCH_TIMEOUT_MS)
    {
        float base_rpm = line_phase_base_rpm();
        float phase_turn_rpm = line_phase_turn_rpm(base_rpm);
        float search_turn_rpm = LINE_LOOKUP_SEARCH_TURN_RPM *
            line_feedback_scale();

        line_output.line_lost = 0u;
        line_output.error = (float)line_last_level * 1000.0f;
        line_output.lookup_level = line_last_level;
        line_apply_targets(base_rpm, phase_turn_rpm,
            search_turn_rpm, dt_s);
        return;
    }

    line_output.left_rpm = 0.0f;
    line_output.right_rpm = 0.0f;
    line_output.pid_turn_rpm = 0.0f;
    line_output.lookup_correction_rpm = 0.0f;
    line_output.curvature_feedforward_rpm = 0.0f;
    line_output.turn_rpm = 0.0f;
    line_output.curve_blend = 0.0f;
    line_output.line_lost = 1u;
    line_applied_base_rpm = 0.0f;
    line_applied_turn_rpm = 0.0f;
}

const line_control_output_t *line_control_get_output(void)
{
    return &line_output;
}
