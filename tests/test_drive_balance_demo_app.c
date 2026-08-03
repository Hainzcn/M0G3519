#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "balance_app.h"
#include "button.h"
#include "control_config.h"
#include "drive_balance_demo_app.h"
#include "encoder.h"
#include "line_control.h"
#include "motor_app.h"

static uint32 mock_now_ms;
static button_id_t mock_button;
static balance_app_status_t mock_balance;
static line_control_output_t mock_line;
static motor_app_mode_enum mock_motor_mode;
static int32 mock_left_count;
static int32 mock_right_count;
static uint32 mock_stop_count;
static float mock_target;
static uint8 mock_target_accepted;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

button_id_t button_get_active(void)
{
    return mock_button;
}

const balance_app_status_t *balance_app_get_status(void)
{
    return &mock_balance;
}

uint8 balance_app_set_target_position_m(float target_position_m)
{
    mock_target = target_position_m;
    return mock_target_accepted;
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

void motor_app_stop(void)
{
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_stop_count++;
}

const line_control_output_t *line_control_get_output(void)
{
    return &mock_line;
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
    mock_button = BUTTON_ID_NONE;
    mock_balance.state = BALANCE_APP_ACTIVE;
    mock_balance.flags = BALANCE_APP_FLAG_ACTIVE;
    mock_balance.estimated_position_m = 0.0f;
    mock_balance.car_imu_accel_valid = 1u;
    mock_line.line_lost = 0u;
    mock_line.marker_detected = 0u;
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_left_count = 0;
    mock_right_count = 0;
    mock_stop_count = 0u;
    mock_target = 99.0f;
    mock_target_accepted = 1u;
    drive_balance_demo_app_init();
}

static void press_button(button_id_t button)
{
    mock_button = button;
    drive_balance_demo_app_process();
    mock_button = BUTTON_ID_NONE;
    drive_balance_demo_app_process();
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
    press_button(BUTTON_ID_SW2);
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    assert(mock_target == 0.0f);
    assert(0u == status->finish_armed);
    assert(0u == mock_stop_count);

    set_distance(4.6f);
    mock_now_ms = 10u;
    drive_balance_demo_app_process();
    assert(0u != status->finish_armed);
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    mock_now_ms = 29u;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CENTER);
    mock_now_ms = 30u;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_COMPLETE);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_LAP_COMPLETE);
    assert(mock_stop_count == 1u);
    assert(status->distance_m > 4.5f);
}

static void test_capture_current_and_user_stop(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    mock_balance.estimated_position_m = 0.043f;
    press_button(BUTTON_ID_SW3);
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_RUNNING_CAPTURED);
    assert(fabsf(mock_target - 0.043f) < 0.0001f);
    mock_balance.estimated_position_m = 0.035f;
    mock_now_ms = 100u;
    drive_balance_demo_app_process();
    assert(fabsf(status->max_abs_error_m - 0.008f) < 0.0001f);
    press_button(BUTTON_ID_SW4);
    assert(status->state == DRIVE_BALANCE_DEMO_ABORTED);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_USER);
}

static void test_start_rejection_and_timeout(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    mock_balance.state = BALANCE_APP_RECOVERY;
    press_button(BUTTON_ID_SW2);
    status = drive_balance_demo_app_get_status();
    assert(status->state == DRIVE_BALANCE_DEMO_IDLE);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_START_REJECTED);
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);

    reset_mocks();
    mock_balance.estimated_position_m = 0.100f;
    press_button(BUTTON_ID_SW3);
    assert(status->state == DRIVE_BALANCE_DEMO_IDLE);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_START_REJECTED);

    reset_mocks();
    press_button(BUTTON_ID_SW2);
    mock_now_ms = BALANCE_DRIVE_DEMO_TIMEOUT_MS;
    drive_balance_demo_app_process();
    assert(status->state == DRIVE_BALANCE_DEMO_TIMEOUT);
    assert(status->stop_reason == DRIVE_BALANCE_STOP_TIMEOUT);
}

static void test_line_and_balance_failures(void)
{
    const drive_balance_demo_status_t *status;

    reset_mocks();
    press_button(BUTTON_ID_SW2);
    status = drive_balance_demo_app_get_status();
    mock_line.line_lost = 1u;
    mock_now_ms = 1u;
    drive_balance_demo_app_process();
    mock_now_ms = 1u + BALANCE_DRIVE_DEMO_LINE_LOSS_TIMEOUT_MS;
    drive_balance_demo_app_process();
    assert(status->stop_reason == DRIVE_BALANCE_STOP_LINE);

    reset_mocks();
    press_button(BUTTON_ID_SW2);
    mock_balance.state = BALANCE_APP_RECOVERY;
    mock_now_ms = 1u;
    drive_balance_demo_app_process();
    assert(status->stop_reason == DRIVE_BALANCE_STOP_BALANCE);

    reset_mocks();
    press_button(BUTTON_ID_SW2);
    mock_balance.car_imu_accel_valid = 0u;
    mock_now_ms = 1u;
    drive_balance_demo_app_process();
    mock_now_ms = 1u + BALANCE_DRIVE_DEMO_IMU_LOSS_TIMEOUT_MS;
    drive_balance_demo_app_process();
    assert(status->stop_reason == DRIVE_BALANCE_STOP_IMU);
}

int main(void)
{
    test_center_lap_ignores_start_marker();
    test_capture_current_and_user_stop();
    test_start_rejection_and_timeout();
    test_line_and_balance_failures();
    puts("drive balance demo tests passed");
    return 0;
}
