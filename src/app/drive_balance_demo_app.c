#include "drive_balance_demo_app.h"

#include <stdio.h>

#include "control_config.h"
#include "encoder.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "line_control.h"
#include "motor_app.h"
#include "balance_simple_app.h"
#include "wheel_speed_control.h"

#define DRIVE_BALANCE_PI (3.14159265358979323846f)

static drive_balance_demo_status_t drive_demo_status;
static uint32 drive_demo_start_ms;
static int32 drive_demo_previous_left_count;
static int32 drive_demo_previous_right_count;
static float drive_demo_marker_distance_m;
static uint32 drive_demo_line_loss_start_ms;
static uint32 drive_demo_imu_loss_start_ms;
static uint8 drive_demo_line_loss_active;
static uint8 drive_demo_imu_loss_active;

typedef struct
{
    float position_m;
    uint8 vision_ready;
    uint8 feedforward_only_ready;
    uint8 imu_valid;
} drive_demo_balance_snapshot_t;

static uint8 drive_demo_feedforward_only;
static uint8 drive_demo_prestart_active;
static uint32 drive_demo_prestart_start_ms;

static void drive_demo_get_planned_accel(uint32 now_ms,
                                         float *planned_accel_mps2,
                                         float *preview_accel_mps2)
{
    const float rpm_s_to_mps2 =
        DRIVE_BALANCE_PI * CHASSIS_WHEEL_DIAMETER_M / 60.0f;
    float preview_s = BALANCE_SIMPLE_CAR_FF_PREVIEW_S;

    *planned_accel_mps2 = 0.0f;
    *preview_accel_mps2 = 0.0f;
    if (0u != drive_demo_prestart_active)
    {
        uint32 elapsed_ms = now_ms - drive_demo_prestart_start_ms;

        if (elapsed_ms > BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS)
        {
            elapsed_ms = BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS;
        }
        preview_s = (float)elapsed_ms * 0.001f;
        *preview_accel_mps2 =
            line_control_get_base_accel_preview_rpm_s(preview_s) *
            rpm_s_to_mps2;
        return;
    }
    if (MOTOR_APP_MODE_LINE_FOLLOW != motor_app_get_mode())
    {
        return;
    }
    *planned_accel_mps2 =
        line_control_get_base_accel_rpm_s() * rpm_s_to_mps2;
    *preview_accel_mps2 = line_control_get_base_accel_preview_rpm_s(
        BALANCE_SIMPLE_CAR_FF_PREVIEW_S) * rpm_s_to_mps2;
}

static float drive_demo_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 drive_demo_running(void)
{
    return ((DRIVE_BALANCE_DEMO_RUNNING_CENTER == drive_demo_status.state) ||
            (DRIVE_BALANCE_DEMO_RUNNING_CAPTURED ==
             drive_demo_status.state) ||
            (DRIVE_BALANCE_DEMO_BRAKING ==
             drive_demo_status.state)) ? 1u : 0u;
}

static void drive_demo_clear_feedforward(void)
{
    balance_simple_app_set_vehicle_accel_components_mps2(
        0.0f, 0.0f, 0.0f, 0u);
}

static void drive_demo_read_balance(uint32 now_ms,
                                    drive_demo_balance_snapshot_t *snapshot)
{
    const balance_simple_status_t *balance =
        balance_simple_app_get_status();
    uint8 capture_ready =
        ((DRIVE_BALANCE_DEMO_PREPARING_CAPTURE ==
          drive_demo_status.state) &&
         (0u != balance_simple_app_capture_ready())) ? 1u : 0u;
    imu_snapshot_t imu;
    float corrected_accel;
    float planned_accel_mps2;
    float preview_accel_mps2;
    snapshot->position_m = balance->estimated_position_m;
    snapshot->vision_ready =
        (((BALANCE_SIMPLE_ACTIVE == balance->state) ||
          (BALANCE_SIMPLE_STATIC_LOCK == balance->state) ||
          (0u != capture_ready)) &&
         (0u != (balance->flags & BALANCE_SIMPLE_FLAG_OBSERVER_VALID)) &&
         (0u != (balance->flags &
                 BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID))) ? 1u : 0u;
    snapshot->feedforward_only_ready =
        ((BALANCE_SIMPLE_WAIT_VISION == balance->state) &&
         (0u != (balance->flags &
                 BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID))) ? 1u : 0u;
    snapshot->imu_valid = 0u;
    imu_get_snapshot(&imu);
    if ((0u != (imu.flags & IMU_FLAG_ACCEL)) &&
        ((now_ms - imu.accel_time_ms) <=
         BALANCE_SIMPLE_CAR_IMU_MAX_AGE_MS))
    {
        corrected_accel =
            (imu.accel.ax - BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2) *
            BALANCE_SIMPLE_CAR_ACCEL_GAIN *
            BALANCE_SIMPLE_CAR_ACCEL_SIGN;
        drive_demo_get_planned_accel(now_ms,
            &planned_accel_mps2, &preview_accel_mps2);
        balance_simple_app_set_vehicle_accel_components_mps2(
            planned_accel_mps2, preview_accel_mps2,
            corrected_accel, 1u);
        snapshot->imu_valid = 1u;
    }
    else
    {
        drive_demo_clear_feedforward();
    }
}

static uint8 drive_demo_set_target(float target_position_m)
{
    return balance_simple_app_set_target_position_m(target_position_m);
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
        "[drive-balance] end=%u,t=%lu,target=%.3f,maxerr=%.3f,dist=%.2f,pass=%u,ok=%u\r\n",
        (unsigned int)drive_demo_status.stop_reason,
        (unsigned long)drive_demo_status.elapsed_ms,
        (double)drive_demo_status.target_position_m,
        (double)drive_demo_status.max_abs_error_m,
        (double)drive_demo_status.distance_m,
        (unsigned int)drive_demo_status.passed_a,
        (unsigned int)drive_demo_status.error_requirement_met);
    heartbeat_hw_uart_send_string(message);
}

static void drive_demo_stop(drive_balance_demo_state_enum state,
                            drive_balance_demo_stop_reason_enum reason,
                            uint32 now_ms)
{
    if (DRIVE_BALANCE_DEMO_COMPLETE == state)
    {
        motor_app_brake();
    }
    else
    {
        motor_app_stop();
    }
    drive_demo_clear_feedforward();
    (void)drive_demo_set_target(0.0f);
    if (0u != drive_demo_feedforward_only)
    {
        (void)balance_simple_app_set_feedforward_only(0u);
        drive_demo_feedforward_only = 0u;
    }
    if (0u == drive_demo_status.passed_a)
    {
        drive_demo_status.elapsed_ms = now_ms - drive_demo_start_ms;
    }
    drive_demo_status.state = state;
    drive_demo_status.stop_reason = reason;
    drive_demo_status.error_requirement_met =
        (drive_demo_status.max_abs_error_m <=
         BALANCE_DRIVE_DEMO_MAX_ERROR_M) ? 1u : 0u;
    drive_demo_line_loss_active = 0u;
    drive_demo_imu_loss_active = 0u;
    drive_demo_prestart_active = 0u;
    drive_demo_log_result();
}

static uint8 drive_demo_start(uint8 capture_current, float line_follow_rpm,
                              uint32 now_ms)
{
    drive_demo_balance_snapshot_t balance;
    float target_position_m = 0.0f;

    drive_demo_read_balance(now_ms, &balance);
    drive_demo_feedforward_only = 0u;
    if ((0u == balance.vision_ready) &&
        (0u != balance.feedforward_only_ready))
    {
        drive_demo_feedforward_only =
            balance_simple_app_set_feedforward_only(1u);
    }
    if ((0u != capture_current) && (0u != balance.vision_ready))
    {
        target_position_m = balance.position_m;
    }

    if (((0u == balance.vision_ready) &&
         (0u == drive_demo_feedforward_only)) ||
        (0u == balance.imu_valid) ||
        (MOTOR_APP_MODE_DISABLED != motor_app_get_mode()) ||
        (0u == grayscale_is_online()) ||
        (drive_demo_abs(target_position_m) >
         BALANCE_TARGET_POSITION_LIMIT_M) ||
        (0u == drive_demo_set_target(target_position_m)))
    {
        drive_demo_clear_feedforward();
        if (0u != drive_demo_feedforward_only)
        {
            (void)balance_simple_app_set_feedforward_only(0u);
            drive_demo_feedforward_only = 0u;
        }
        drive_demo_status.state = (0u != capture_current) ?
            DRIVE_BALANCE_DEMO_PREPARING_CAPTURE :
            DRIVE_BALANCE_DEMO_IDLE;
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
    drive_demo_status.approach_active = 0u;
    drive_demo_status.passed_a = 0u;
    drive_demo_status.elapsed_ms = 0u;
    drive_demo_status.target_position_m = target_position_m;
    drive_demo_status.max_abs_error_m = (0u != balance.vision_ready) ?
        drive_demo_abs(target_position_m - balance.position_m) : 0.0f;
    drive_demo_status.error_requirement_met =
        (drive_demo_status.max_abs_error_m <=
         BALANCE_DRIVE_DEMO_MAX_ERROR_M) ? 1u : 0u;
    drive_demo_status.distance_m = 0.0f;
    drive_demo_marker_distance_m = 0.0f;
    drive_demo_start_ms = now_ms;
    drive_demo_previous_left_count = encoder_get_left_total_count();
    drive_demo_previous_right_count = encoder_get_right_total_count();
    drive_demo_line_loss_active = 0u;
    drive_demo_imu_loss_active = 0u;
    motor_app_set_base_rpm(line_follow_rpm);
    drive_demo_prestart_active = 1u;
    drive_demo_prestart_start_ms = now_ms;
    drive_demo_read_balance(now_ms, &balance);
    if (0u != drive_demo_feedforward_only)
    {
        heartbeat_hw_uart_send_string(
            "[drive-balance] vision off; feedforward-only prestart\r\n");
    }
    else
    {
        heartbeat_hw_uart_send_string((0u != capture_current) ?
            "[drive-balance] SW3 captured target; prestart\r\n" :
            "[drive-balance] SW2 center target; prestart\r\n");
    }
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

static void drive_demo_process_running(uint32 now_ms)
{
    drive_demo_balance_snapshot_t balance;
    const line_control_output_t *line = line_control_get_output();
    float error;

    drive_demo_read_balance(now_ms, &balance);
    if (0u != drive_demo_prestart_active)
    {
        if ((0u == balance.vision_ready) &&
            !((0u != drive_demo_feedforward_only) &&
              (0u != balance.feedforward_only_ready)))
        {
            drive_demo_stop(DRIVE_BALANCE_DEMO_FAULT_STOP,
                            DRIVE_BALANCE_STOP_BALANCE, now_ms);
            return;
        }
        if (0u != drive_demo_loss_timed_out(
                (uint8)(0u == balance.imu_valid),
                &drive_demo_imu_loss_active,
                &drive_demo_imu_loss_start_ms,
                BALANCE_DRIVE_DEMO_IMU_LOSS_TIMEOUT_MS, now_ms))
        {
            drive_demo_stop(DRIVE_BALANCE_DEMO_FAULT_STOP,
                            DRIVE_BALANCE_STOP_IMU, now_ms);
            return;
        }
        drive_demo_status.elapsed_ms = 0u;
        if ((now_ms - drive_demo_prestart_start_ms) <
            BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS)
        {
            return;
        }
        motor_app_set_line_follow_enabled(1u);
        drive_demo_prestart_active = 0u;
        drive_demo_start_ms = now_ms;
        drive_demo_previous_left_count = encoder_get_left_total_count();
        drive_demo_previous_right_count = encoder_get_right_total_count();
        drive_demo_line_loss_active = 0u;
        drive_demo_imu_loss_active = 0u;
        drive_demo_read_balance(now_ms, &balance);
        heartbeat_hw_uart_send_string(
            "[drive-balance] chassis launch\r\n");
        return;
    }
    drive_demo_status.elapsed_ms = now_ms - drive_demo_start_ms;
    drive_demo_update_distance();
    error = drive_demo_abs(drive_demo_status.target_position_m -
                           balance.position_m);
    if ((0u != balance.vision_ready) &&
        (error > drive_demo_status.max_abs_error_m))
    {
        drive_demo_status.max_abs_error_m = error;
    }
    if ((0u != balance.vision_ready) &&
        (error > BALANCE_DRIVE_DEMO_MAX_ERROR_M))
    {
        drive_demo_status.error_requirement_met = 0u;
    }

    if ((0u == balance.vision_ready) &&
        !((0u != drive_demo_feedforward_only) &&
          (0u != balance.feedforward_only_ready)))
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
            (uint8)(0u == balance.imu_valid),
            &drive_demo_imu_loss_active,
            &drive_demo_imu_loss_start_ms,
            BALANCE_DRIVE_DEMO_IMU_LOSS_TIMEOUT_MS, now_ms))
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_FAULT_STOP,
                        DRIVE_BALANCE_STOP_IMU, now_ms);
        return;
    }
    if (DRIVE_BALANCE_DEMO_BRAKING == drive_demo_status.state)
    {
        if ((drive_demo_status.distance_m -
             drive_demo_marker_distance_m) >=
            BALANCE_DRIVE_DEMO_POST_MARKER_DISTANCE_M)
        {
            drive_demo_stop(DRIVE_BALANCE_DEMO_COMPLETE,
                            DRIVE_BALANCE_STOP_LAP_COMPLETE, now_ms);
        }
        return;
    }
    if (drive_demo_status.elapsed_ms > BALANCE_DRIVE_DEMO_TIMEOUT_MS)
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_TIMEOUT,
                        DRIVE_BALANCE_STOP_TIMEOUT, now_ms);
        return;
    }
    if (drive_demo_status.distance_m >=
        NO_LOAD_LAP_MARKER_MIN_DISTANCE_M)
    {
        drive_demo_status.finish_armed = 1u;
    }
    if ((0u == drive_demo_status.approach_active) &&
        (drive_demo_status.distance_m >=
         BALANCE_DRIVE_DEMO_APPROACH_DISTANCE_M))
    {
        drive_demo_status.approach_active = 1u;
        motor_app_set_base_rpm(BALANCE_DRIVE_DEMO_APPROACH_RPM);
        heartbeat_hw_uart_send_string("[drive-balance] approach A\r\n");
    }
    if ((0u != drive_demo_status.finish_armed) &&
        (line->active_count >= NO_LOAD_LAP_MARKER_MIN_ACTIVE_COUNT))
    {
        drive_demo_status.passed_a = 1u;
        drive_demo_status.error_requirement_met =
            (drive_demo_status.max_abs_error_m <=
             BALANCE_DRIVE_DEMO_MAX_ERROR_M) ? 1u : 0u;
        drive_demo_status.state = DRIVE_BALANCE_DEMO_BRAKING;
        drive_demo_marker_distance_m = drive_demo_status.distance_m;
        motor_app_set_rapid_brake_enabled(1u);
        motor_app_set_base_rpm_immediate(
            line_control_get_base_rpm());
        drive_demo_read_balance(now_ms, &balance);
        heartbeat_hw_uart_send_string(
            "[drive-balance] marker latched; post=230mm\r\n");
    }
}

void drive_balance_demo_app_init(void)
{
    drive_demo_status.state = DRIVE_BALANCE_DEMO_IDLE;
    drive_demo_status.stop_reason = DRIVE_BALANCE_STOP_NONE;
    drive_demo_status.finish_armed = 0u;
    drive_demo_status.approach_active = 0u;
    drive_demo_status.passed_a = 0u;
    drive_demo_status.error_requirement_met = 1u;
    drive_demo_status.elapsed_ms = 0u;
    drive_demo_status.target_position_m = 0.0f;
    drive_demo_status.max_abs_error_m = 0.0f;
    drive_demo_status.distance_m = 0.0f;
    drive_demo_start_ms = heartbeat_get_ms();
    drive_demo_marker_distance_m = 0.0f;
    drive_demo_previous_left_count = encoder_get_left_total_count();
    drive_demo_previous_right_count = encoder_get_right_total_count();
    drive_demo_line_loss_active = 0u;
    drive_demo_imu_loss_active = 0u;
    drive_demo_feedforward_only = 0u;
    drive_demo_prestart_active = 0u;
    drive_demo_prestart_start_ms = drive_demo_start_ms;
}

void drive_balance_demo_app_process(void)
{
    if (0u != drive_demo_running())
    {
        drive_demo_process_running(heartbeat_get_ms());
    }
}

uint8 drive_balance_demo_app_start_center(void)
{
    if ((0u != drive_demo_running()) ||
        (DRIVE_BALANCE_DEMO_PREPARING_CAPTURE == drive_demo_status.state))
    {
        return 0u;
    }
    return drive_demo_start(
        0u, TRACK_MODE_4_LINE_FOLLOW_RPM, heartbeat_get_ms());
}

uint8 drive_balance_demo_app_prepare_captured(void)
{
    if ((0u != drive_demo_running()) ||
        (MOTOR_APP_MODE_DISABLED != motor_app_get_mode()) ||
        (0u == balance_simple_app_prepare_capture()))
    {
        return 0u;
    }
    drive_demo_status.state = DRIVE_BALANCE_DEMO_PREPARING_CAPTURE;
    drive_demo_status.stop_reason = DRIVE_BALANCE_STOP_NONE;
    drive_demo_status.finish_armed = 0u;
    drive_demo_status.approach_active = 0u;
    drive_demo_status.passed_a = 0u;
    drive_demo_status.elapsed_ms = 0u;
    drive_demo_status.target_position_m = 0.0f;
    drive_demo_status.max_abs_error_m = 0.0f;
    drive_demo_status.distance_m = 0.0f;
    heartbeat_hw_uart_send_string(
        "[drive-balance] mode 5 level; waiting SW3 capture\r\n");
    return 1u;
}

uint8 drive_balance_demo_app_capture_ready(void)
{
    return ((DRIVE_BALANCE_DEMO_PREPARING_CAPTURE ==
             drive_demo_status.state) &&
            (0u != balance_simple_app_capture_ready())) ? 1u : 0u;
}

uint8 drive_balance_demo_app_start_captured(void)
{
    if (0u == drive_balance_demo_app_capture_ready())
    {
        heartbeat_hw_uart_send_string(
            "[drive-balance] SW3 capture rejected: level/vision not ready\r\n");
        return 0u;
    }
    return drive_demo_start(
        1u, TRACK_MODE_5_LINE_FOLLOW_RPM, heartbeat_get_ms());
}

void drive_balance_demo_app_stop(void)
{
    if (DRIVE_BALANCE_DEMO_PREPARING_CAPTURE == drive_demo_status.state)
    {
        balance_simple_app_cancel_capture();
        drive_demo_status.state = DRIVE_BALANCE_DEMO_ABORTED;
        drive_demo_status.stop_reason = DRIVE_BALANCE_STOP_USER;
        return;
    }
    if (0u != drive_demo_running())
    {
        drive_demo_stop(DRIVE_BALANCE_DEMO_ABORTED,
                        DRIVE_BALANCE_STOP_USER, heartbeat_get_ms());
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
