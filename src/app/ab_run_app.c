#include "ab_run_app.h"

#include <stdio.h>

#include "balance_simple_app.h"
#include "control_config.h"
#include "encoder.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"

#define AB_RUN_PI_F       (3.14159265358979323846f)
#define AB_RUN_INVALID_AGE (0xFFFFFFFFu)

static ab_run_status_t ab_status;
static uint32 ab_start_ms;
static uint32 ab_line_loss_start_ms;
static uint32 ab_imu_loss_start_ms;
static int32 ab_previous_left_count;
static int32 ab_previous_right_count;
static uint8 ab_line_loss_active;
static uint8 ab_imu_loss_active;
static uint8 ab_feedforward_only;
static uint8 ab_force_straight_active;

static void ab_get_planned_accel(float *planned_accel_mps2,
                                 float *preview_accel_mps2)
{
    const float rpm_s_to_mps2 =
        AB_RUN_PI_F * CHASSIS_WHEEL_DIAMETER_M / 60.0f;

    *planned_accel_mps2 = 0.0f;
    *preview_accel_mps2 = 0.0f;
    if (MOTOR_APP_MODE_LINE_FOLLOW != motor_app_get_mode())
    {
        return;
    }
    *planned_accel_mps2 =
        line_control_get_base_accel_rpm_s() * rpm_s_to_mps2;
    *preview_accel_mps2 = line_control_get_base_accel_preview_rpm_s(
        BALANCE_SIMPLE_CAR_FF_PREVIEW_S) * rpm_s_to_mps2;
}

static float ab_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ab_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static uint8 ab_active(void)
{
    return (AB_RUN_RUNNING == ab_status.state) ? 1u : 0u;
}

static void ab_update_distance(void)
{
    int32 left_count = encoder_get_left_total_count();
    int32 right_count = encoder_get_right_total_count();
    float center_counts = 0.5f *
        ((float)(left_count - ab_previous_left_count) *
             WHEEL_LEFT_ENCODER_SIGN +
         (float)(right_count - ab_previous_right_count) *
             WHEEL_RIGHT_ENCODER_SIGN);
    float delta_m = center_counts *
        (AB_RUN_PI_F * CHASSIS_WHEEL_DIAMETER_M) /
        (float)ENCODER_COUNTS_PER_WHEEL_REV;

    ab_previous_left_count = left_count;
    ab_previous_right_count = right_count;
    if (delta_m > 0.0f)
    {
        ab_status.distance_m += delta_m;
    }
}

static uint8 ab_update_feedforward(uint32 now_ms)
{
    imu_snapshot_t imu;
    float corrected;
    float planned_accel_mps2;
    float preview_accel_mps2;
    float feedforward;

    imu_get_snapshot(&imu);
    ab_status.imu_accel_mps2 = imu.accel.ax;
    ab_status.imu_age_ms = AB_RUN_INVALID_AGE;
    ab_status.imu_valid = 0u;
    ab_status.feedforward_accel_mps2 = 0.0f;
    if (0u == (imu.flags & IMU_FLAG_ACCEL))
    {
        balance_simple_app_set_vehicle_accel_components_mps2(
            0.0f, 0.0f, 0.0f, 0u);
        return 0u;
    }

    ab_status.imu_age_ms = now_ms - imu.accel_time_ms;
    if (ab_status.imu_age_ms > BALANCE_SIMPLE_CAR_IMU_MAX_AGE_MS)
    {
        balance_simple_app_set_vehicle_accel_components_mps2(
            0.0f, 0.0f, 0.0f, 0u);
        return 0u;
    }
    corrected = (imu.accel.ax - BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2) *
        BALANCE_SIMPLE_CAR_ACCEL_GAIN * BALANCE_SIMPLE_CAR_ACCEL_SIGN;
    corrected = ab_clamp(corrected,
        -BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2,
        BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2);
    ab_get_planned_accel(&planned_accel_mps2, &preview_accel_mps2);
    feedforward = preview_accel_mps2 +
        BALANCE_SIMPLE_CAR_FF_IMU_CORRECTION_GAIN *
            (corrected - planned_accel_mps2);
    feedforward = ab_clamp(feedforward,
        -BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2,
        BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2);
    ab_status.feedforward_accel_mps2 = feedforward;
    ab_status.imu_valid = 1u;
    balance_simple_app_set_vehicle_accel_components_mps2(
        planned_accel_mps2, preview_accel_mps2, corrected, 1u);
    return 1u;
}

static void ab_log_result(void)
{
    char message[144];

    snprintf(message, sizeof(message),
        "[ab-run] end=%u,t=%lu,dist=%.3f,maxerr=%.3f,pass=%u,ok=%u\r\n",
        (unsigned int)ab_status.state,
        (unsigned long)ab_status.elapsed_ms,
        (double)ab_status.distance_m,
        (double)ab_status.max_abs_error_m,
        (unsigned int)ab_status.passed_b,
        (unsigned int)ab_status.error_requirement_met);
    heartbeat_hw_uart_send_string(message);
}

static void ab_finish(ab_run_state_enum state, uint32 now_ms)
{
    motor_app_stop();
    balance_simple_app_set_vehicle_accel_mps2(0.0f, 0u);
    if (0u != ab_feedforward_only)
    {
        (void)balance_simple_app_set_feedforward_only(0u);
        ab_feedforward_only = 0u;
    }
    if (0u == ab_status.passed_b)
    {
        ab_status.elapsed_ms = now_ms - ab_start_ms;
    }
    ab_status.state = state;
    ab_status.error_requirement_met =
        (ab_status.max_abs_error_m <= AB_RUN_MAX_ERROR_M) ? 1u : 0u;
    ab_line_loss_active = 0u;
    ab_imu_loss_active = 0u;
    ab_force_straight_active = 0u;
    ab_log_result();
}

static uint8 ab_timed_out(uint8 condition, uint8 *active,
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

void ab_run_app_init(void)
{
    ab_status.state = AB_RUN_IDLE;
    ab_status.passed_b = 0u;
    ab_status.error_requirement_met = 1u;
    ab_status.imu_valid = 0u;
    ab_status.elapsed_ms = 0u;
    ab_status.imu_age_ms = AB_RUN_INVALID_AGE;
    ab_status.distance_m = 0.0f;
    ab_status.max_abs_error_m = 0.0f;
    ab_status.imu_accel_mps2 = 0.0f;
    ab_status.feedforward_accel_mps2 = 0.0f;
    ab_start_ms = heartbeat_get_ms();
    ab_line_loss_start_ms = 0u;
    ab_imu_loss_start_ms = 0u;
    ab_previous_left_count = encoder_get_left_total_count();
    ab_previous_right_count = encoder_get_right_total_count();
    ab_line_loss_active = 0u;
    ab_imu_loss_active = 0u;
    ab_feedforward_only = 0u;
    ab_force_straight_active = 0u;
}

uint8 ab_run_app_start(void)
{
    const balance_simple_status_t *balance =
        balance_simple_app_get_status();
    uint32 now_ms = heartbeat_get_ms();
    uint8 vision_ready =
        (((BALANCE_SIMPLE_ACTIVE == balance->state) ||
          (BALANCE_SIMPLE_STATIC_LOCK == balance->state)) &&
         (0u != (balance->flags & BALANCE_SIMPLE_FLAG_OBSERVER_VALID))) ?
        1u : 0u;
    uint8 feedforward_only_ready =
        ((BALANCE_SIMPLE_WAIT_VISION == balance->state) &&
         (0u != (balance->flags &
                 BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID))) ? 1u : 0u;

    ab_feedforward_only = 0u;
    if ((0u == vision_ready) && (0u != feedforward_only_ready))
    {
        ab_feedforward_only =
            balance_simple_app_set_feedforward_only(1u);
    }
    if (((0u == vision_ready) && (0u == ab_feedforward_only)) ||
        (MOTOR_APP_MODE_DISABLED != motor_app_get_mode()) ||
        (0u == grayscale_is_online()) ||
        (0u == ab_update_feedforward(now_ms)) ||
        (0u == balance_simple_app_set_target_position_m(0.0f)))
    {
        if (0u != ab_feedforward_only)
        {
            (void)balance_simple_app_set_feedforward_only(0u);
            ab_feedforward_only = 0u;
        }
        balance_simple_app_set_vehicle_accel_mps2(0.0f, 0u);
        return 0u;
    }

    ab_status.state = AB_RUN_RUNNING;
    ab_status.passed_b = 0u;
    ab_status.error_requirement_met = 1u;
    ab_status.elapsed_ms = 0u;
    ab_status.distance_m = 0.0f;
    ab_status.max_abs_error_m = (0u != vision_ready) ?
        ab_abs(balance->estimated_position_m) : 0.0f;
    ab_start_ms = now_ms;
    ab_previous_left_count = encoder_get_left_total_count();
    ab_previous_right_count = encoder_get_right_total_count();
    ab_line_loss_active = 0u;
    ab_imu_loss_active = 0u;
    ab_force_straight_active = 0u;
    motor_app_set_base_rpm(TRACK_MODE_3_LINE_FOLLOW_RPM);
    motor_app_set_line_follow_enabled(1u);
    (void)ab_update_feedforward(now_ms);
    heartbeat_hw_uart_send_string((0u != ab_feedforward_only) ?
        "[ab-run] start; vision off; feedforward only\r\n" :
        "[ab-run] start\r\n");
    return 1u;
}

void ab_run_app_process(void)
{
    const balance_simple_status_t *balance;
    const line_control_output_t *line;
    float error;
    uint32 now_ms;
    uint8 imu_valid;

    if (0u == ab_active())
    {
        return;
    }
    now_ms = heartbeat_get_ms();
    balance = balance_simple_app_get_status();
    imu_valid = ab_update_feedforward(now_ms);
    ab_update_distance();
    error = ab_abs(balance->estimated_position_m);
    if ((0u != (balance->flags & BALANCE_SIMPLE_FLAG_OBSERVER_VALID)) &&
        (error > ab_status.max_abs_error_m))
    {
        ab_status.max_abs_error_m = error;
    }

    if (((BALANCE_SIMPLE_ACTIVE != balance->state) &&
         (BALANCE_SIMPLE_STATIC_LOCK != balance->state)) &&
        !((0u != ab_feedforward_only) &&
          (BALANCE_SIMPLE_WAIT_VISION == balance->state) &&
          (0u != (balance->flags &
                  BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID))))
    {
        ab_finish(AB_RUN_BALANCE_FAULT, now_ms);
        return;
    }
    if (0u != ab_timed_out((uint8)(0u == imu_valid),
            &ab_imu_loss_active, &ab_imu_loss_start_ms,
            AB_RUN_IMU_LOSS_TIMEOUT_MS, now_ms))
    {
        ab_finish(AB_RUN_IMU_LOST, now_ms);
        return;
    }

    ab_status.elapsed_ms = now_ms - ab_start_ms;
    if (ab_status.distance_m >= AB_RUN_TARGET_DISTANCE_M)
    {
        ab_status.passed_b = 1u;
        ab_status.error_requirement_met =
            (ab_status.max_abs_error_m <= AB_RUN_MAX_ERROR_M) ? 1u : 0u;
        ab_finish(AB_RUN_COMPLETE, now_ms);
        return;
    }
    if ((0u == ab_force_straight_active) &&
        (ab_status.distance_m >= AB_RUN_FORCE_STRAIGHT_DISTANCE_M))
    {
        /* This also rejects the right-shifting line beyond 1.50 m. */
        motor_app_set_speed_test(TRACK_MODE_3_LINE_FOLLOW_RPM,
                                 TRACK_MODE_3_LINE_FOLLOW_RPM);
        ab_force_straight_active = 1u;
        ab_line_loss_active = 0u;
    }
    if (0u == ab_force_straight_active)
    {
        line = line_control_get_output();
        if (0u != ab_timed_out(line->line_lost, &ab_line_loss_active,
                &ab_line_loss_start_ms, AB_RUN_LINE_LOSS_TIMEOUT_MS, now_ms))
        {
            ab_finish(AB_RUN_LINE_LOST, now_ms);
            return;
        }
    }
    if (ab_status.elapsed_ms >= AB_RUN_TIMEOUT_MS)
    {
        ab_finish(AB_RUN_TIMEOUT, now_ms);
        return;
    }
}

void ab_run_app_stop(void)
{
    if (0u != ab_active())
    {
        ab_finish(AB_RUN_USER_STOP, heartbeat_get_ms());
    }
}

uint8 ab_run_app_is_running(void)
{
    return ab_active();
}

const ab_run_status_t *ab_run_app_get_status(void)
{
    return &ab_status;
}
