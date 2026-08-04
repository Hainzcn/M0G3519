#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "balance_simple_app.h"
#include "control_config.h"
#include "control_config_legacy.h"
#include "drive_balance_demo_app.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"

static uint32 mock_now_ms;
static balance_simple_status_t mock_balance;
static imu_snapshot_t mock_imu;
static line_control_output_t mock_line;
static motor_app_mode_enum mock_motor_mode;
static int32 mock_left_count;
static int32 mock_right_count;
static uint32 mock_stop_count;
static float mock_target;
static uint8 mock_target_accepted;
static uint8 mock_grayscale_online;
static uint8 mock_feedforward_valid;
static float mock_feedforward_accel_mps2;
static float mock_planned_accel_mps2;
static float mock_preview_accel_mps2;
static float mock_imu_accel_mps2;
static float mock_base_rpm;
static wheel_speed_control_status_t mock_wheel;
static uint8 mock_feedforward_only;
static uint8 mock_capture_prepare_result;
static uint8 mock_capture_ready;
static uint32 mock_capture_cancel_count;
static float mock_line_accel_rpm_s;
static float mock_line_preview_accel_rpm_s;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

const balance_simple_status_t *balance_simple_app_get_status(void)
{
    return &mock_balance;
}

uint8 balance_simple_app_set_target_position_m(float target_position_m)
{
    mock_target = target_position_m;
    return mock_target_accepted;
}

void balance_simple_app_set_vehicle_accel_mps2(float accel_mps2, uint8 valid)
{
    mock_feedforward_accel_mps2 = accel_mps2;
    mock_feedforward_valid = valid;
}

void balance_simple_app_set_vehicle_accel_components_mps2(
    float planned_accel_mps2, float preview_accel_mps2,
    float imu_accel_mps2, uint8 valid)
{
    mock_planned_accel_mps2 = planned_accel_mps2;
    mock_preview_accel_mps2 = preview_accel_mps2;
    mock_imu_accel_mps2 = imu_accel_mps2;
    mock_feedforward_accel_mps2 = preview_accel_mps2 +
        BALANCE_SIMPLE_CAR_FF_IMU_CORRECTION_GAIN *
            (imu_accel_mps2 - planned_accel_mps2);
    mock_feedforward_valid = valid;
}

uint8 balance_simple_app_set_feedforward_only(uint8 enabled)
{
    mock_feedforward_only = enabled;
    return 1u;
}

uint8 balance_simple_app_prepare_capture(void)
{
    return mock_capture_prepare_result;
}

uint8 balance_simple_app_capture_ready(void)
{
    return mock_capture_ready;
}

void balance_simple_app_cancel_capture(void)
{
    mock_capture_cancel_count++;
    mock_capture_ready = 0u;
}

const wheel_speed_control_status_t *wheel_speed_control_get_status(void)
{
    return &mock_wheel;
}

void imu_get_snapshot(imu_snapshot_t *snapshot)
{
    *snapshot = mock_imu;
}

uint8 grayscale_is_online(void)
{
    return mock_grayscale_online;
}

motor_app_mode_enum motor_app_get_mode(void)
{
    return mock_motor_mode;
}

void motor_app_set_line_follow_enabled(uint8 enabled)
{
    mock_motor_mode = (0u != enabled) ?
        MOTOR_APP_MODE_LINE_FOLLOW : MOTOR_APP_MODE_DISABLED;
}

void motor_app_set_base_rpm(float rpm)
{
    mock_base_rpm = rpm;
}

void motor_app_stop(void)
{
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_stop_count++;
}

const line_control_output_t *line_control_get_output(void)
{
    return &mock_line;
}

float line_control_get_base_accel_rpm_s(void)
{
    return mock_line_accel_rpm_s;
}

float line_control_get_base_accel_preview_rpm_s(float preview_s)
{
    assert(preview_s == BALANCE_SIMPLE_CAR_FF_PREVIEW_S);
    return mock_line_preview_accel_rpm_s;
}

int32 encoder_get_left_total_count(void)
{
    return mock_left_count;
}

int32 encoder_get_right_total_count(void)
{
    return mock_right_count;
}

static void reset_mocks(void)
{
    mock_now_ms = 0u;
    mock_balance.state = BALANCE_SIMPLE_ACTIVE;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_OBSERVER_VALID |
                         BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    mock_balance.estimated_position_m = 0.0f;
    mock_imu.flags = IMU_FLAG_ACCEL;
    mock_imu.accel.ax = BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2;
    mock_imu.accel_time_ms = mock_now_ms;
    mock_line.line_lost = 0u;
    mock_line.marker_detected = 0u;
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_left_count = 0;
    mock_right_count = 0;
    mock_stop_count = 0u;
    mock_target = 99.0f;
    mock_target_accepted = 1u;
    mock_grayscale_online = 1u;
    mock_feedforward_valid = 0u;
    mock_feedforward_accel_mps2 = 0.0f;
    mock_planned_accel_mps2 = 0.0f;
    mock_preview_accel_mps2 = 0.0f;
    mock_imu_accel_mps2 = 0.0f;
    mock_base_rpm = 0.0f;
    mock_wheel.kinematics_valid = 0u;
    mock_wheel.planned_accel_mps2 = 0.0f;
    mock_feedforward_only = 0u;
    mock_capture_prepare_result = 1u;
    mock_capture_ready = 1u;
    mock_capture_cancel_count = 0u;
    mock_line_accel_rpm_s = 0.0f;
    mock_line_preview_accel_rpm_s = 0.0f;
    drive_balance_demo_app_init();
}

static void set_distance(float distance_m)
{
    float counts = distance_m * (float)ENCODER_COUNTS_PER_WHEEL_REV /
        (3.14159265358979323846f * CHASSIS_WHEEL_DIAMETER_M);
    mock_left_count = (int32)counts;
    mock_right_count = -(int32)counts;
}

static void test_center_lap_ignores_start_marker(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    mock_line.marker_detected = 1u;
    assert(0u != drive_balance_demo_app_start_center());
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    assert(mock_target == 0.0f);
    assert(0u == status->finish_armed);
    assert(0u == mock_stop_count);

    set_distance(4.6f);
    mock_now_ms = 10u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(0u != status->finish_armed);
    assert(fabsf(mock_base_rpm - TRACK_MODE_4_LINE_FOLLOW_RPM) < 0.001f);
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    mock_now_ms = 29u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    mock_now_ms = 30u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_BRAKING);
    assert(status->passed_a != 0u);
    assert(mock_stop_count == 0u);
    assert(mock_motor_mode == MOTOR_APP_MODE_LINE_FOLLOW);
    assert(mock_base_rpm == 0.0f);
    assert(status->distance_m > 4.5f);
    assert(mock_feedforward_valid != 0u);
    mock_now_ms += BALANCE_DRIVE_DEMO_BRAKE_HOLD_MS;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_COMPLETE);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_LAP_COMPLETE);
    assert(mock_stop_count == 1u);
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);
    assert(mock_feedforward_valid == 0u);
}

static void test_planned_acceleration_is_previewed_and_imu_corrects_residual(void)
{
    const float rpm_s_to_mps2 =
        3.14159265358979323846f * CHASSIS_WHEEL_DIAMETER_M / 60.0f;

    reset_mocks();
    mock_imu.accel.ax = BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2 + 0.2f;

    assert(0u != drive_balance_demo_app_start_center());
    mock_line_accel_rpm_s = 0.6f / rpm_s_to_mps2;
    mock_line_preview_accel_rpm_s = 0.8f / rpm_s_to_mps2;
    mock_now_ms += 10u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();

    assert(mock_feedforward_valid != 0u);
    assert(fabsf(mock_planned_accel_mps2 - 0.6f) < 0.0001f);
    assert(fabsf(mock_preview_accel_mps2 - 0.8f) < 0.0001f);
    assert(fabsf(mock_imu_accel_mps2 - 0.2f) < 0.0001f);
    assert(fabsf(mock_feedforward_accel_mps2 - 0.6f) < 0.0001f);
}

static void test_capture_current_and_user_stop(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    mock_balance.estimated_position_m = 0.043f;
    assert(0u != drive_balance_demo_app_prepare_captured());
    assert(drive_balance_demo_app_get_status()->state ==
           DRIVE_BALANCE_DEMO_PREPARING_CAPTURE);
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);
    assert(0u != drive_balance_demo_app_start_captured());
    assert(fabsf(mock_base_rpm - TRACK_MODE_5_LINE_FOLLOW_RPM) < 0.001f);
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CAPTURED);
    assert(fabsf(mock_target - 0.043f) < 0.0001f);
    mock_balance.estimated_position_m = 0.035f;
    mock_now_ms = 100u;
    drive_balance_demo_app_process();
    assert(fabsf(status->max_abs_error_m - 0.008f) < 0.0001f);
    drive_balance_demo_app_stop();
    assert(status->state == DRIVE_BALANCE_DEMO_ABORTED);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_USER);
    assert(mock_target == 0.0f);
}

static void test_start_rejection_and_timeout(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_SAFE_RETURN;
    assert(0u == drive_balance_demo_app_start_center());
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_IDLE);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_START_REJECTED);
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);

    reset_mocks();
    mock_balance.estimated_position_m = 0.100f;
    assert(0u != drive_balance_demo_app_prepare_captured());
    assert(0u == drive_balance_demo_app_start_captured());
    assert(status->state == DRIVE_BALANCE_DEMO_PREPARING_CAPTURE);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_START_REJECTED);

    reset_mocks();
    assert(0u != drive_balance_demo_app_start_center());
    mock_now_ms = BALANCE_DRIVE_DEMO_TIMEOUT_MS + 1u;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_TIMEOUT);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_TIMEOUT);
}

static void test_vision_off_only_starts_center_mode_feedforward_only(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_WAIT_VISION;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    mock_balance.estimated_position_m = 0.043f;
    assert(0u != drive_balance_demo_app_start_center());
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    assert(mock_feedforward_only != 0u);
    assert(status->target_position_m == 0.0f);
    mock_now_ms += 10u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(drive_balance_demo_app_is_running() != 0u);
    drive_balance_demo_app_stop();
    assert(mock_feedforward_only == 0u);

    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_WAIT_VISION;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    mock_balance.estimated_position_m = 0.043f;
    mock_capture_ready = 0u;
    assert(0u != drive_balance_demo_app_prepare_captured());
    assert(0u == drive_balance_demo_app_start_captured());
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_PREPARING_CAPTURE);
    assert(mock_feedforward_only == 0u);

    drive_balance_demo_app_stop();
    assert(mock_capture_cancel_count == 1u);
    assert(status->state == DRIVE_BALANCE_DEMO_ABORTED);
}

static void test_line_and_balance_failures(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    assert(0u != drive_balance_demo_app_start_center());
    status = drive_balance_demo_app_get_status();
    mock_line.line_lost = 1u;
    mock_now_ms = 1u;
    drive_balance_demo_app_process();
    mock_now_ms = 1u + BALANCE_DRIVE_DEMO_LINE_LOSS_TIMEOUT_MS;
    drive_balance_demo_app_process();
    assert(status->stop_reason == DRIVE_BALANCE_STOP_LINE);

    reset_mocks();
    assert(0u != drive_balance_demo_app_start_center());
    mock_balance.state = BALANCE_SIMPLE_SAFE_RETURN;
    mock_now_ms = 1u;
    drive_balance_demo_app_process();
    assert(status->stop_reason == DRIVE_BALANCE_STOP_BALANCE);

    reset_mocks();
    assert(0u != drive_balance_demo_app_start_center());
    mock_imu.flags = 0u;
    mock_now_ms = 1u;
    drive_balance_demo_app_process();
    mock_now_ms = 1u + BALANCE_DRIVE_DEMO_IMU_LOSS_TIMEOUT_MS;
    drive_balance_demo_app_process();
    assert(status->stop_reason == DRIVE_BALANCE_STOP_IMU);
}

static void test_approach_speed_and_error_requirement(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    assert(0u != drive_balance_demo_app_start_center());
    status = drive_balance_demo_app_get_status();
    set_distance(BALANCE_DRIVE_DEMO_APPROACH_DISTANCE_M + 0.1f);
    mock_balance.estimated_position_m = 0.011f;
    mock_now_ms = 1000u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(status->approach_active != 0u);
    assert(fabsf(mock_base_rpm - BALANCE_DRIVE_DEMO_APPROACH_RPM) < 0.001f);
    assert(status->error_requirement_met == 0u);

    mock_line.marker_detected = 1u;
    mock_now_ms += 1u;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    mock_now_ms += BALANCE_DRIVE_DEMO_MARKER_DEBOUNCE_MS;
    mock_imu.accel_time_ms = mock_now_ms;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_BRAKING);
    assert(status->error_requirement_met == 0u);
}

int main(void)
{
    test_center_lap_ignores_start_marker();
    test_planned_acceleration_is_previewed_and_imu_corrects_residual();
    test_capture_current_and_user_stop();
    test_start_rejection_and_timeout();
    test_vision_off_only_starts_center_mode_feedforward_only();
    test_line_and_balance_failures();
    test_approach_speed_and_error_requirement();
    puts("drive balance demo tests passed");
    return 0;
}
