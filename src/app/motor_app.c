#include "motor_app.h"

#include <stdio.h>

#include "control_config.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "line_control.h"
#include "motor.h"
#include "wheel_speed_control.h"

#define MOTOR_APP_SCAN_TIMEOUT_MS       (30u)
#define MOTOR_APP_DEBUG_PERIOD_MS       (250u)

static motor_app_mode_enum motor_app_mode;
static uint32 motor_app_last_control_ms;
static uint32 motor_app_last_scan_ms;
static uint32 motor_app_last_scan_sequence;
static uint32 motor_app_last_debug_ms;
static float motor_app_test_left_rpm;
static float motor_app_test_right_rpm;

static float motor_app_clamp(float value, float min_value, float max_value)
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

static void motor_app_send_debug(uint32 now_ms)
{
    char message[176];
    const line_control_output_t *line;
    const wheel_speed_control_status_t *wheel;

    if ((now_ms - motor_app_last_debug_ms) < MOTOR_APP_DEBUG_PERIOD_MS)
    {
        return;
    }

    motor_app_last_debug_ms = now_ms;
    line = line_control_get_output();
    wheel = wheel_speed_control_get_status();
    snprintf(message, sizeof(message),
        "[ctl] %u,v=%d,k=%d,h=%d,q=%02X,p=%u,d=%d,g=%d,e=%d,n=%u,c=%d,b=%d,"
        "t=%d/%d,m=%d/%d,u=%d/%d,s=%u%u\r\n",
        (unsigned int)motor_app_mode,
        (int)line_control_get_base_rpm(),
        (int)(line->speed_scale * 1000.0f),
        (int)(line->feedback_scale * 1000.0f),
        (unsigned int)line->sensor_mask,
        (unsigned int)line->phase,
        (int)(line->phase_distance_m * 1000.0f),
        (int)(line->curve_blend * 1000.0f),
        (int)line->error,
        (unsigned int)line->active_count,
        (int)line->lookup_correction_rpm,
        (int)line->curvature_feedforward_rpm,
        (int)wheel->left_target_rpm, (int)wheel->right_target_rpm,
        (int)wheel->left_measured_rpm, (int)wheel->right_measured_rpm,
        (int)wheel->left_duty, (int)wheel->right_duty,
        (unsigned int)wheel->left_saturated,
        (unsigned int)wheel->right_saturated);
    heartbeat_hw_uart_send_string(message);
}

void motor_app_init(void)
{
    motor_init();
    line_control_init();
    wheel_speed_control_init();

    motor_app_mode = MOTOR_APP_MODE_DISABLED;
    motor_app_last_control_ms = heartbeat_get_ms();
    motor_app_last_scan_ms = motor_app_last_control_ms;
    motor_app_last_scan_sequence = grayscale_get_scan_sequence();
    motor_app_last_debug_ms = motor_app_last_control_ms;
    motor_app_test_left_rpm = 0.0f;
    motor_app_test_right_rpm = 0.0f;

    if (0u != MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO)
    {
        motor_app_set_right_circle_demo(RIGHT_CIRCLE_DEMO_CENTER_RPM);
    }
    else if (0u != MOTOR_APP_AUTO_START_LINE_FOLLOW)
    {
        motor_app_set_line_follow_enabled(1u);
    }
}

void motor_app_process(void)
{
    const line_control_output_t *line;
    uint32 now_ms = heartbeat_get_ms();
    uint32 elapsed_ms = now_ms - motor_app_last_control_ms;
    uint32 scan_sequence;

    if (elapsed_ms < CHASSIS_CONTROL_PERIOD_MS)
    {
        return;
    }

    motor_app_last_control_ms = now_ms;

    /* Stop for one cycle after a long stall; use the real encoder interval. */
    if (elapsed_ms > 50u)
    {
        line_control_reset();
        wheel_speed_control_reset();
        wheel_speed_control_update(elapsed_ms, 0u);
        return;
    }

    if (MOTOR_APP_MODE_DISABLED == motor_app_mode)
    {
        wheel_speed_control_update(elapsed_ms, 0u);
        return;
    }

    if ((MOTOR_APP_MODE_SPEED_TEST == motor_app_mode) ||
        (MOTOR_APP_MODE_RIGHT_CIRCLE_DEMO == motor_app_mode))
    {
        wheel_speed_control_set_target(motor_app_test_left_rpm,
                                       motor_app_test_right_rpm);
        wheel_speed_control_update(elapsed_ms, 1u);
        motor_app_send_debug(now_ms);
        return;
    }

    scan_sequence = grayscale_get_scan_sequence();
    if (scan_sequence != motor_app_last_scan_sequence)
    {
        motor_app_last_scan_sequence = scan_sequence;
        motor_app_last_scan_ms = now_ms;
        line_control_update(grayscale_get_values(), now_ms,
                            (float)elapsed_ms * 0.001f);
    }
    else if ((now_ms - motor_app_last_scan_ms) > MOTOR_APP_SCAN_TIMEOUT_MS)
    {
        line_control_reset();
    }

    line = line_control_get_output();
    if (0u != line->line_lost)
    {
        wheel_speed_control_update(elapsed_ms, 0u);
    }
    else
    {
        wheel_speed_control_set_target(line->left_rpm, line->right_rpm);
        wheel_speed_control_update(elapsed_ms, 1u);
    }
    motor_app_send_debug(now_ms);
}

void motor_app_stop(void)
{
    motor_app_mode = MOTOR_APP_MODE_DISABLED;
    line_control_reset();
    wheel_speed_control_reset();
    motor_stop();
}

void motor_app_set_line_follow_enabled(uint8 enabled)
{
    if (0u == enabled)
    {
        motor_app_stop();
        return;
    }

    line_control_reset();
    wheel_speed_control_reset();
    motor_app_last_scan_ms = heartbeat_get_ms();
    motor_app_last_scan_sequence = grayscale_get_scan_sequence();
    motor_app_mode = MOTOR_APP_MODE_LINE_FOLLOW;
}

void motor_app_set_base_rpm(float base_rpm)
{
    line_control_set_base_rpm(base_rpm);
}

void motor_app_set_speed_test(float left_rpm, float right_rpm)
{
    wheel_speed_control_reset();
    motor_app_test_left_rpm = left_rpm;
    motor_app_test_right_rpm = right_rpm;
    motor_app_mode = MOTOR_APP_MODE_SPEED_TEST;
}

void motor_app_set_right_circle_demo(float center_rpm)
{
    const float turn_ratio = CHASSIS_WHEEL_TRACK_M /
        RIGHT_CIRCLE_DIAMETER_M;
    const float max_center_rpm = WHEEL_TARGET_RPM_LIMIT /
        (1.0f + turn_ratio);
    float turn_rpm;

    center_rpm = motor_app_clamp(center_rpm, 0.0f,
        max_center_rpm);
    turn_rpm = center_rpm * turn_ratio;

    wheel_speed_control_reset();
    motor_app_test_left_rpm = motor_app_clamp(center_rpm + turn_rpm,
        0.0f, WHEEL_TARGET_RPM_LIMIT);
    motor_app_test_right_rpm = motor_app_clamp(center_rpm - turn_rpm,
        0.0f, WHEEL_TARGET_RPM_LIMIT);
    motor_app_mode = MOTOR_APP_MODE_RIGHT_CIRCLE_DEMO;
}

motor_app_mode_enum motor_app_get_mode(void)
{
    return motor_app_mode;
}
