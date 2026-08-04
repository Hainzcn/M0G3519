#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ab_run_app.h"
#include "balance_simple_app.h"
#include "control_config.h"
#include "encoder.h"
#include "grayscale.h"
#include "imu.h"
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"

static uint32 mock_now_ms;
static balance_simple_status_t mock_balance;
static motor_app_mode_enum mock_motor_mode;
static line_control_output_t mock_line;
static imu_snapshot_t mock_imu;
static int32 mock_left_count;
static int32 mock_right_count;
static uint8 mock_grayscale_online;
static float mock_target;
static float mock_feedforward_accel;
static float mock_planned_accel;
static float mock_preview_accel;
static float mock_imu_accel;
static uint8 mock_feedforward_valid;
static uint32 mock_stop_count;
static wheel_speed_control_status_t mock_wheel;
static uint8 mock_feedforward_only;
static uint32 mock_speed_test_count;
static float mock_left_target_rpm;
static float mock_right_target_rpm;
static float mock_line_accel_rpm_s;
static float mock_line_preview_accel_rpm_s;
static float mock_line_preview_s;
static uint32 mock_line_start_count;

uint32 heartbeat_get_ms(void) { return mock_now_ms; }
void heartbeat_hw_uart_send_string(const char *message) { (void)message; }
const balance_simple_status_t *balance_simple_app_get_status(void)
{
    return &mock_balance;
}
uint8 balance_simple_app_set_target_position_m(float target)
{
    mock_target = target;
    return 1u;
}
void balance_simple_app_set_vehicle_accel_mps2(float accel, uint8 valid)
{
    mock_feedforward_accel = accel;
    mock_feedforward_valid = valid;
}
void balance_simple_app_set_vehicle_accel_components_mps2(
    float planned, float preview, float imu, uint8 valid)
{
    mock_planned_accel = planned;
    mock_preview_accel = preview;
    mock_imu_accel = imu;
    mock_feedforward_accel = preview +
        BALANCE_SIMPLE_CAR_FF_IMU_CORRECTION_GAIN * (imu - planned);
    mock_feedforward_valid = valid;
}
uint8 balance_simple_app_set_feedforward_only(uint8 enabled)
{
    mock_feedforward_only = enabled;
    return 1u;
}
motor_app_mode_enum motor_app_get_mode(void) { return mock_motor_mode; }
void motor_app_set_base_rpm(float rpm)
{
    assert(rpm == TRACK_MODE_3_LINE_FOLLOW_RPM);
}
void motor_app_set_line_follow_enabled(uint8 enabled)
{
    mock_motor_mode = (0u != enabled) ?
        MOTOR_APP_MODE_LINE_FOLLOW : MOTOR_APP_MODE_DISABLED;
    if (0u != enabled)
    {
        mock_line_start_count++;
    }
}
void motor_app_set_speed_test(float left_rpm, float right_rpm)
{
    mock_motor_mode = MOTOR_APP_MODE_SPEED_TEST;
    mock_left_target_rpm = left_rpm;
    mock_right_target_rpm = right_rpm;
    mock_speed_test_count++;
}
void motor_app_stop(void)
{
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_stop_count++;
}
int32 encoder_get_left_total_count(void) { return mock_left_count; }
int32 encoder_get_right_total_count(void) { return mock_right_count; }
uint8 grayscale_is_online(void) { return mock_grayscale_online; }
const line_control_output_t *line_control_get_output(void) { return &mock_line; }
float line_control_get_base_accel_rpm_s(void)
{
    return mock_line_accel_rpm_s;
}
float line_control_get_base_accel_preview_rpm_s(float preview_s)
{
    mock_line_preview_s = preview_s;
    return mock_line_preview_accel_rpm_s;
}
const wheel_speed_control_status_t *wheel_speed_control_get_status(void)
{
    return &mock_wheel;
}
void imu_get_snapshot(imu_snapshot_t *snapshot) { *snapshot = mock_imu; }

static void reset_mocks(void)
{
    memset(&mock_balance, 0, sizeof(mock_balance));
    memset(&mock_line, 0, sizeof(mock_line));
    memset(&mock_imu, 0, sizeof(mock_imu));
    memset(&mock_wheel, 0, sizeof(mock_wheel));
    mock_now_ms = 100u;
    mock_balance.state = BALANCE_SIMPLE_ACTIVE;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_OBSERVER_VALID |
                         BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_imu.flags = IMU_FLAG_ACCEL;
    mock_imu.accel.ax = BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2 + 0.5f;
    mock_imu.accel_time_ms = mock_now_ms;
    mock_grayscale_online = 1u;
    mock_target = 99.0f;
    mock_feedforward_accel = 0.0f;
    mock_planned_accel = 0.0f;
    mock_preview_accel = 0.0f;
    mock_imu_accel = 0.0f;
    mock_feedforward_valid = 0u;
    mock_stop_count = 0u;
    mock_feedforward_only = 0u;
    mock_speed_test_count = 0u;
    mock_left_target_rpm = 0.0f;
    mock_right_target_rpm = 0.0f;
    mock_line_accel_rpm_s = 0.0f;
    mock_line_preview_accel_rpm_s = 0.0f;
    mock_line_preview_s = 0.0f;
    mock_line_start_count = 0u;
    mock_left_count = 0;
    mock_right_count = 0;
    ab_run_app_init();
}

static void complete_prestart(void)
{
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);
    mock_now_ms += BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(mock_motor_mode == MOTOR_APP_MODE_LINE_FOLLOW);
    assert(mock_line_start_count == 1u);
    assert(fabsf(mock_line_preview_s -
        BALANCE_SIMPLE_CAR_FF_PREVIEW_S) < 0.0001f);
}

static void test_planned_acceleration_is_previewed_and_imu_corrects_residual(void)
{
    const float rpm_s_to_mps2 =
        3.14159265358979323846f * CHASSIS_WHEEL_DIAMETER_M / 60.0f;

    reset_mocks();
    mock_imu.accel.ax = BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2 + 0.2f;

    assert(0u != ab_run_app_start());
    mock_line_accel_rpm_s = 0.6f / rpm_s_to_mps2;
    mock_line_preview_accel_rpm_s = 0.8f / rpm_s_to_mps2;
    mock_now_ms += BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS / 2u;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);
    assert(mock_planned_accel == 0.0f);
    assert(fabsf(mock_line_preview_s -
        (0.001f *
         (float)(BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS / 2u))) <
        0.0001f);
    complete_prestart();

    assert(fabsf(mock_planned_accel - 0.6f) < 0.0001f);
    assert(fabsf(mock_preview_accel - 0.8f) < 0.0001f);
    assert(fabsf(mock_imu_accel - 0.2f) < 0.0001f);
    assert(fabsf(mock_feedforward_accel - 0.6f) < 0.0001f);
}

static void set_distance(float distance_m)
{
    float counts = distance_m * (float)ENCODER_COUNTS_PER_WHEEL_REV /
        (3.14159265358979323846f * CHASSIS_WHEEL_DIAMETER_M);
    mock_left_count = (int32)counts;
    mock_right_count = -(int32)counts;
}

static void test_switches_to_straight_then_completes_at_target(void)
{
    const ab_run_status_t *status;

    reset_mocks();
    assert(0u != ab_run_app_start());
    assert(mock_target == 0.0f);
    assert(mock_feedforward_valid != 0u);
    assert(fabsf(mock_feedforward_accel - 0.25f) < 0.0001f);
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);
    complete_prestart();

    set_distance(AB_RUN_FORCE_STRAIGHT_DISTANCE_M + 0.01f);
    mock_now_ms += 3000u;
    mock_imu.accel_time_ms = mock_now_ms;
    mock_balance.estimated_position_m = 0.008f;
    ab_run_app_process();
    status = ab_run_app_get_status();
    assert(status->state == AB_RUN_RUNNING);
    assert(status->passed_b == 0u);
    assert(status->elapsed_ms == 3000u);
    assert(mock_motor_mode == MOTOR_APP_MODE_SPEED_TEST);
    assert(mock_speed_test_count == 1u);
    assert(mock_left_target_rpm == TRACK_MODE_3_LINE_FOLLOW_RPM);
    assert(mock_right_target_rpm == TRACK_MODE_3_LINE_FOLLOW_RPM);
    assert(mock_stop_count == 0u);
    assert(mock_feedforward_valid != 0u);

    mock_line.lookup_level = 4;
    mock_line.right_active_count = 4u;
    mock_line.turn_rpm = 42.0f;
    mock_line.left_rpm = 102.0f;
    mock_line.right_rpm = 18.0f;
    set_distance(AB_RUN_IGNORE_RIGHT_SHIFT_DISTANCE_M + 0.01f);
    mock_now_ms += 10u;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(status->state == AB_RUN_RUNNING);
    assert(mock_speed_test_count == 1u);
    assert(mock_left_target_rpm == mock_right_target_rpm);
    assert(mock_stop_count == 0u);

    mock_line.line_lost = 1u;
    mock_now_ms += AB_RUN_LINE_LOSS_TIMEOUT_MS + 1u;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(status->state == AB_RUN_RUNNING);

    set_distance(AB_RUN_TARGET_DISTANCE_M + 0.01f);
    mock_now_ms += 100u;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(status->state == AB_RUN_COMPLETE);
    assert(status->passed_b != 0u);
    assert(status->error_requirement_met != 0u);
    assert(mock_stop_count == 1u);
    assert(mock_feedforward_valid == 0u);
}

static void test_timeout_and_imu_loss_stop(void)
{
    const ab_run_status_t *status;

    reset_mocks();
    assert(0u != ab_run_app_start());
    complete_prestart();
    mock_now_ms += AB_RUN_TIMEOUT_MS;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    status = ab_run_app_get_status();
    assert(status->state == AB_RUN_TIMEOUT);

    reset_mocks();
    assert(0u != ab_run_app_start());
    complete_prestart();
    mock_imu.flags = 0u;
    mock_now_ms += 1u;
    ab_run_app_process();
    mock_now_ms += AB_RUN_IMU_LOSS_TIMEOUT_MS;
    ab_run_app_process();
    assert(status->state == AB_RUN_IMU_LOST);
}

static void test_vision_off_starts_feedforward_only_but_still_requires_imu(void)
{
    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_WAIT_VISION;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    assert(0u != ab_run_app_start());
    assert(mock_feedforward_only != 0u);
    mock_now_ms += 10u;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(ab_run_app_is_running() != 0u);
    ab_run_app_stop();
    assert(mock_feedforward_only == 0u);

    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_WAIT_VISION;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    mock_imu.flags = 0u;
    assert(0u == ab_run_app_start());
}

int main(void)
{
    test_switches_to_straight_then_completes_at_target();
    test_planned_acceleration_is_previewed_and_imu_corrects_residual();
    test_timeout_and_imu_loss_stop();
    test_vision_off_starts_feedforward_only_but_still_requires_imu();
    puts("AB run app tests passed");
    return 0;
}
