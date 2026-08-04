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
static uint8 mock_feedforward_valid;
static uint32 mock_stop_count;

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
motor_app_mode_enum motor_app_get_mode(void) { return mock_motor_mode; }
void motor_app_set_base_rpm(float rpm) { assert(rpm == AB_RUN_CRUISE_RPM); }
void motor_app_set_line_follow_enabled(uint8 enabled)
{
    mock_motor_mode = (0u != enabled) ?
        MOTOR_APP_MODE_LINE_FOLLOW : MOTOR_APP_MODE_DISABLED;
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
void imu_get_snapshot(imu_snapshot_t *snapshot) { *snapshot = mock_imu; }

static void reset_mocks(void)
{
    memset(&mock_balance, 0, sizeof(mock_balance));
    memset(&mock_line, 0, sizeof(mock_line));
    memset(&mock_imu, 0, sizeof(mock_imu));
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
    mock_feedforward_valid = 0u;
    mock_stop_count = 0u;
    mock_left_count = 0;
    mock_right_count = 0;
    ab_run_app_init();
}

static void set_distance(float distance_m)
{
    float counts = distance_m * (float)ENCODER_COUNTS_PER_WHEEL_REV /
        (3.14159265358979323846f * CHASSIS_WHEEL_DIAMETER_M);
    mock_left_count = (int32)counts;
    mock_right_count = -(int32)counts;
}

static void test_passes_b_and_holds_feedforward_through_braking(void)
{
    const ab_run_status_t *status;

    reset_mocks();
    assert(0u != ab_run_app_start());
    assert(mock_target == 0.0f);
    assert(mock_feedforward_valid != 0u);
    assert(fabsf(mock_feedforward_accel - 0.5f) < 0.0001f);

    set_distance(1.53f);
    mock_now_ms = 3100u;
    mock_imu.accel_time_ms = mock_now_ms;
    mock_balance.estimated_position_m = 0.008f;
    ab_run_app_process();
    status = ab_run_app_get_status();
    assert(status->state == AB_RUN_BRAKING);
    assert(status->passed_b != 0u);
    assert(status->elapsed_ms == 3000u);
    assert(status->error_requirement_met != 0u);
    assert(mock_stop_count == 1u);
    assert(mock_feedforward_valid != 0u);

    mock_now_ms += AB_RUN_BRAKE_HOLD_MS;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    assert(status->state == AB_RUN_COMPLETE);
    assert(mock_stop_count == 1u);
    assert(mock_feedforward_valid == 0u);
}

static void test_timeout_and_imu_loss_stop(void)
{
    const ab_run_status_t *status;

    reset_mocks();
    assert(0u != ab_run_app_start());
    mock_now_ms += AB_RUN_TIMEOUT_MS;
    mock_imu.accel_time_ms = mock_now_ms;
    ab_run_app_process();
    status = ab_run_app_get_status();
    assert(status->state == AB_RUN_TIMEOUT);

    reset_mocks();
    assert(0u != ab_run_app_start());
    mock_imu.flags = 0u;
    mock_now_ms += 1u;
    ab_run_app_process();
    mock_now_ms += AB_RUN_IMU_LOSS_TIMEOUT_MS;
    ab_run_app_process();
    assert(status->state == AB_RUN_IMU_LOST);
}

static void test_start_requires_ready_balance_and_imu(void)
{
    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_WAIT_VISION;
    assert(0u == ab_run_app_start());
    reset_mocks();
    mock_imu.flags = 0u;
    assert(0u == ab_run_app_start());
}

int main(void)
{
    test_passes_b_and_holds_feedforward_through_braking();
    test_timeout_and_imu_loss_stop();
    test_start_requires_ready_balance_and_imu();
    puts("AB run app tests passed");
    return 0;
}
