#include "drive_balance_demo_app.h"

#include <stdio.h>

#include "balance_app.h"
#include "button.h"
#include "control_config.h"
#include "control_config_legacy.h"
#include "encoder.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "line_control.h"
#include "motor_app.h"

#define DRIVE_BALANCE_PI (3.14159265358979323846f)

static drive_balance_demo_status_t drive_demo_status;
static button_id_t drive_demo_previous_button;
static uint32 drive_demo_start_ms;
static int32 drive_demo_previous_left_count;
static int32 drive_demo_previous_right_count;
static uint32 drive_demo_marker_start_ms;
static uint32 drive_demo_line_loss_start_ms;
static uint32 drive_demo_imu_loss_start_ms;
static uint8 drive_demo_marker_active;
static uint8 drive_demo_line_loss_active;
static uint8 drive_demo_imu_loss_active;

static float drive_demo_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 drive_demo_running(void)
{
    return ((DRIVE_BALANCE_DEMO_RUNNING_CENTER == drive_demo_status.state) ||
            (DRIVE_BALANCE_DEMO_RUNNING_CAPTURED ==
             drive_demo_status.state)) ? 1u : 0u;
}

static button_id_t drive_demo_take_button_edge(void)
{
    button_id_t active = button_get_active();
    button_id_t edge = BUTTON_ID_NONE;

    if ((BUTTON_ID_NONE != active) &&
        (active != drive_demo_previous_button))
    {
        edge = active;
    }
    drive_demo_previous_button = active;
    return edge;
}

static void drive_demo_update_distance(void)
{
    int32 left_count = encoder_get_left_total_count();
    int32 right_count = encoder_get_right_total_count();
    int32 left_delta = left_count - drive_demo_previous_left_count;
    int32 right_delta = right_count - drive_demo_previous_right_count;
    float center_counts;
    float distance_delta_m;

    drive_demo_previous_left_count = left_count;
    drive_demo_previous_right_count = right_count;
    center_counts = 0.5f *
        ((float)left_delta * WHEEL_LEFT_ENCODER_SIGN +
         (float)right_delta * WHEEL_RIGHT_ENCODER_SIGN);
    distance_delta_m = center_counts *
        (DRIVE_BALANCE_PI * CHASSIS_WHEEL_DIAMETER_M) /
        (float)ENCODER_COUNTS_PER_WHEEL_REV;
    if (distance_delta_m > 0.0f)
    {
        drive_demo_status.distance_m += distance_delta_m;
    }
}

static void drive_demo_log_result(void)
{
    char message[176];

    snprintf(message, sizeof(message),
        "[drive-balance] end=%u,t=%lu,target=%.3f,maxerr=%.3f,dist=%.2f\r\n",
        (unsigned int)drive_demo_status.stop_reason,
        (unsigned long)drive_demo_status.elapsed_ms,
        (double)drive_demo_status.target_position_m,
        (double)drive_demo_status.max_abs_error_m,
        (double)drive_demo_status.distance_m);
    heartbeat_hw_uart_send_string(message);
}

static void drive_demo_stop(drive_balance_demo_state_enum state,
                            drive_balance_demo_stop_reason_enum reason,
                            uint32 now_ms)
{
    motor_app_stop();
    drive_demo_status.elapsed_ms = now_ms - drive_demo_start_ms;
    drive_demo_status.state = state;
    drive_demo_status.stop_reason = reason;
    drive_demo_marker_active = 0u;
    drive_demo_line_loss_active = 0u;
    drive_demo_imu_loss_active = 0u;
    drive_demo_log_result();
}

static uint8 drive_demo_start(uint8 capture_current, uint32 now_ms)
{
    const balance_app_status_t *balance = balance_app_get_status();
    float target_position_m = (0u != capture_current) ?
        balance->estimated_position_m : 0.0f;

    if ((BALANCE_APP_ACTIVE != balance->state) ||
        (0u != (balance->flags & BALANCE_APP_FLAG_SEQUENCE_ACTIVE)) ||
        (0u == balance->car_imu_accel_valid) ||
        (MOTOR_APP_MODE_DISABLED != motor_app_get_mode()) ||
        (drive_demo_abs(target_position_m) >
         BALANCE_TARGET_POSITION_LIMIT_M) ||
        (0u == balance_app_set_target_position_m(target_position_m)))
    {
        drive_demo_status.state = DRIVE_BALANCE_DEMO_IDLE;
        drive_demo_status.stop_reason = DRIVE_BALANCE_STOP_START_REJECTED;
        heartbeat_hw_uart_send_string(
            "[drive-balance] start rejected: balance/imu/chassis/target\r\n");
        return 0u;
    }

    drive_demo_status.state = (0u != capture_current) ?
        DRIVE_BALANCE_DEMO_RUNNING_CAPTURED :
        DRIVE_BALANCE_DEMO_RUNNING_CENTER;
    drive_demo_status.stop_reason = DRIVE_BALANCE_STOP_NONE;
    drive_demo_status.finish_armed = 0u;
    drive_demo_status.elapsed_ms = 0u;
    drive_demo_status.target_position_m = target_position_m;
    drive_demo_status.max_abs_error_m =
        drive_demo_abs(target_position_m - balance->estimated_position_m);
    drive_demo_status.distance_m = 0.0f;
    drive_demo_start_ms = now_ms;
    drive_demo_previous_left_count = encoder_get_left_total_count();
    drive_demo_previous_right_count = encoder_get_right_total_count();
    drive_demo_marker_active = 0u;
    drive_demo_line_loss_active = 0u;
    drive_demo_imu_loss_active = 0u;
    motor_app_set_line_follow_enabled(1u);
    heartbeat_hw_uart_send_string((0u != capture_current) ?
        "[drive-balance] SW3 captured target; lap start\r\n" :
        "[drive-balance] SW2 center target; lap start\r\n");
    return 1u;
}

static uint8 drive_demo_loss_timed_out(uint8 condition, uint8 *active,
                                       uint32 *start_ms, uint32 timeout_ms,
                                       uint32 now_ms)
{
    if (0u == condition)
    {
        *active = 0u;
        return 0u;
    }
    if (0u == *active)
    {
        *active = 1u;
        *start_ms = now_ms;
        return 0u;
    }
    return ((now_ms - *start_ms) >= timeout_ms) ? 1u : 0u;
}

static void drive_demo_process_running(uint32 now_ms,
                                       button_id_t button_edge)
{
    const balance_app_status_t *balance = balance_app_get_status();
    const line_control_output_t *line = line_control_get_output();
    float error;

    drive_demo_status.elapsed_ms = now_ms - drive_demo_start_ms;
    drive_demo_update_distance();
    error = drive_demo_abs(drive_demo_status.target_position_m -
                           balance->estimated_position_m);
    if (error > drive_demo_status.max_abs_error_m)
    {
        drive_demo_status.max_abs_error_m = error;
    }

    if (BUTTON_ID_SW4 == button_edge)
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_ABORTED,
                        DRIVE_BALANCE_STOP_USER, now_ms);
        return;
    }
    if (BALANCE_APP_ACTIVE != balance->state)
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_FAULT_STOP,
                        DRIVE_BALANCE_STOP_BALANCE, now_ms);
        return;
    }
    if (0u != drive_demo_loss_timed_out(
            line->line_lost, &drive_demo_line_loss_active,
            &drive_demo_line_loss_start_ms,
            BALANCE_DRIVE_DEMO_LINE_LOSS_TIMEOUT_MS, now_ms))
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_FAULT_STOP,
                        DRIVE_BALANCE_STOP_LINE, now_ms);
        return;
    }
    if (0u != drive_demo_loss_timed_out(
            (uint8)(0u == balance->car_imu_accel_valid),
            &drive_demo_imu_loss_active,
            &drive_demo_imu_loss_start_ms,
            BALANCE_DRIVE_DEMO_IMU_LOSS_TIMEOUT_MS, now_ms))
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_FAULT_STOP,
                        DRIVE_BALANCE_STOP_IMU, now_ms);
        return;
    }
    if (drive_demo_status.elapsed_ms >= BALANCE_DRIVE_DEMO_TIMEOUT_MS)
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_TIMEOUT,
                        DRIVE_BALANCE_STOP_TIMEOUT, now_ms);
        return;
    }
    if (drive_demo_status.distance_m >=
        BALANCE_DRIVE_DEMO_LAP_ARM_DISTANCE_M)
    {
        drive_demo_status.finish_armed = 1u;
    }
    if ((0u != drive_demo_status.finish_armed) &&
        (0u != line->marker_detected))
    {
        if (0u == drive_demo_marker_active)
        {
            drive_demo_marker_active = 1u;
            drive_demo_marker_start_ms = now_ms;
        }
        else if ((now_ms - drive_demo_marker_start_ms) >=
                 BALANCE_DRIVE_DEMO_MARKER_DEBOUNCE_MS)
        {
            drive_demo_stop(DRIVE_BALANCE_DEMO_COMPLETE,
                            DRIVE_BALANCE_STOP_LAP_COMPLETE, now_ms);
        }
    }
    else
    {
        drive_demo_marker_active = 0u;
    }
}

void drive_balance_demo_app_init(void)
{
    drive_demo_status.state = DRIVE_BALANCE_DEMO_IDLE;
    drive_demo_status.stop_reason = DRIVE_BALANCE_STOP_NONE;
    drive_demo_status.finish_armed = 0u;
    drive_demo_status.elapsed_ms = 0u;
    drive_demo_status.target_position_m = 0.0f;
    drive_demo_status.max_abs_error_m = 0.0f;
    drive_demo_status.distance_m = 0.0f;
    drive_demo_previous_button = BUTTON_ID_NONE;
    drive_demo_start_ms = heartbeat_get_ms();
    drive_demo_previous_left_count = encoder_get_left_total_count();
    drive_demo_previous_right_count = encoder_get_right_total_count();
    drive_demo_marker_active = 0u;
    drive_demo_line_loss_active = 0u;
    drive_demo_imu_loss_active = 0u;
}

void drive_balance_demo_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();
    button_id_t button_edge = drive_demo_take_button_edge();

    if (0u != drive_demo_running())
    {
        drive_demo_process_running(now_ms, button_edge);
        return;
    }
    if (BUTTON_ID_SW2 == button_edge)
    {
        (void)drive_demo_start(0u, now_ms);
    }
    else if (BUTTON_ID_SW3 == button_edge)
    {
        (void)drive_demo_start(1u, now_ms);
    }
}

uint8 drive_balance_demo_app_is_running(void)
{
    return drive_demo_running();
}

const drive_balance_demo_status_t *drive_balance_demo_app_get_status(void)
{
    return &drive_demo_status;
}
