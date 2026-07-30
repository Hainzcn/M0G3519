#include "line_control.h"

#include "control_config.h"
#include "control_pid.h"

static const int16 line_sensor_weight[GRAYSCALE_CHANNELS] =
{
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500,
};

static control_pid_t line_pid;
static line_control_output_t line_output;
static float line_base_rpm;
static float line_last_error;
static float line_filtered_error;
static uint32 line_last_valid_ms;
static uint32 line_right_detect_start_ms;
static uint8 line_has_valid;
static uint8 line_filter_initialized;
static uint8 line_right_detect_pending;

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
    const control_pid_config_t config =
    {
        .kp = LINE_KP,
        .ki = LINE_KI,
        .kd = LINE_KD,
        .integral_limit = LINE_INTEGRAL_LIMIT,
        .output_limit = LINE_TURN_RPM_LIMIT,
    };

    control_pid_init(&line_pid, &config);
    line_base_rpm = LINE_BASE_RPM_DEFAULT;
    line_control_reset();
}

void line_control_reset(void)
{
    control_pid_reset(&line_pid);
    line_output.left_rpm = 0.0f;
    line_output.right_rpm = 0.0f;
    line_output.error = 0.0f;
    line_output.pid_turn_rpm = 0.0f;
    line_output.curvature_feedforward_rpm = 0.0f;
    line_output.turn_rpm = 0.0f;
    line_output.active_count = 0u;
    line_output.right_active_count = 0u;
    line_output.right_curve_detected = 0u;
    line_output.line_valid = 0u;
    line_output.marker_detected = 0u;
    line_output.line_lost = 1u;
    line_last_error = 0.0f;
    line_filtered_error = 0.0f;
    line_last_valid_ms = 0u;
    line_right_detect_start_ms = 0u;
    line_has_valid = 0u;
    line_filter_initialized = 0u;
    line_right_detect_pending = 0u;
}

void line_control_set_base_rpm(float base_rpm)
{
    line_base_rpm = line_clamp(base_rpm, 0.0f, WHEEL_TARGET_RPM_LIMIT);
}

void line_control_update(const uint8 values[GRAYSCALE_CHANNELS],
                         uint32 now_ms, float dt_s)
{
    int32 weighted_sum = 0;
    uint8 index;
    uint8 active_count = 0u;
    uint8 black_count = 0u;
    uint8 right_active_count = 0u;
    float error;
    float speed_scale;
    float base_rpm;
    float turn_rpm;
    float pid_turn_rpm;
    float curvature_feedforward_rpm;
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
            weighted_sum += line_sensor_weight[index];
            active_count++;
        }

        if (LINE_BLACK_ACTIVE_LEVEL == values[index])
        {
            black_count++;
            if ((index >= LINE_RIGHT_SENSOR_FIRST_INDEX) &&
                (index <= LINE_RIGHT_SENSOR_LAST_INDEX))
            {
                right_active_count++;
            }
        }
    }

    line_output.active_count = active_count;
    line_output.right_active_count = right_active_count;
    line_output.marker_detected =
        (black_count >= LINE_SENSOR_MARKER_MIN_COUNT) ? 1u : 0u;

    if ((0u != right_active_count) &&
        (0u == line_output.marker_detected))
    {
        if (0u == line_right_detect_pending)
        {
            line_right_detect_start_ms = now_ms;
            line_right_detect_pending = 1u;
        }
        if ((now_ms - line_right_detect_start_ms) >=
            LINE_RIGHT_CURVE_DETECT_MS)
        {
            line_output.right_curve_detected = 1u;
        }
    }
    else
    {
        line_right_detect_pending = 0u;
        line_output.right_curve_detected = 0u;
    }

    if (0u != active_count)
    {
        error = (float)weighted_sum / (float)active_count;
        if (0u == line_filter_initialized)
        {
            line_filtered_error = error;
            line_filter_initialized = 1u;
        }
        else
        {
            line_filtered_error += LINE_ERROR_FILTER_ALPHA *
                (error - line_filtered_error);
        }
        error = line_filtered_error;
        line_last_error = line_filtered_error;
        line_last_valid_ms = now_ms;
        line_has_valid = 1u;
        line_output.line_valid = 1u;
        line_output.line_lost = 0u;
    }
    else if ((0u != line_has_valid) &&
             ((now_ms - line_last_valid_ms) <= LINE_LOST_HOLD_MS))
    {
        error = line_last_error;
        line_output.line_valid = 0u;
        line_output.line_lost = 0u;
    }
    else
    {
        control_pid_reset(&line_pid);
        line_output.left_rpm = 0.0f;
        line_output.right_rpm = 0.0f;
        line_output.error = line_last_error;
        line_output.pid_turn_rpm = 0.0f;
        line_output.curvature_feedforward_rpm = 0.0f;
        line_output.turn_rpm = 0.0f;
        line_output.line_valid = 0u;
        line_output.line_lost = 1u;
        return;
    }

    pid_turn_rpm = LINE_STEERING_SIGN *
        control_pid_step(&line_pid, error, 0.0f, dt_s);
    speed_scale = line_clamp(line_abs(error) / 3500.0f, 0.0f, 1.0f);
    base_rpm = line_base_rpm -
        (line_base_rpm - LINE_MIN_RPM_DEFAULT) * speed_scale;
    curvature_feedforward_rpm = 0.0f;
    if (0u != line_output.right_curve_detected)
    {
        curvature_feedforward_rpm = base_rpm * CHASSIS_WHEEL_TRACK_M /
            RIGHT_CIRCLE_DIAMETER_M;
    }
    turn_rpm = line_clamp(pid_turn_rpm + curvature_feedforward_rpm,
        -LINE_TURN_RPM_LIMIT, LINE_TURN_RPM_LIMIT);

    line_output.error = error;
    line_output.pid_turn_rpm = pid_turn_rpm;
    line_output.curvature_feedforward_rpm = curvature_feedforward_rpm;
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
