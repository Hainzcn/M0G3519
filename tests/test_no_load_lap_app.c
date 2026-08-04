#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "encoder.h"
#include "grayscale.h"
#include "line_control.h"
#include "motor_app.h"
#include "no_load_lap_app.h"

static uint32 mock_now_ms;
static uint8 mock_grayscale_online;
static int32 mock_left_count;
static int32 mock_right_count;
static line_control_output_t mock_line;
static motor_app_mode_enum mock_motor_mode;
static float mock_base_rpm;
static uint32 mock_line_start_count;
static uint32 mock_stop_count;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

uint8 grayscale_is_online(void)
{
    return mock_grayscale_online;
}

int32 encoder_get_left_total_count(void)
{
    return mock_left_count;
}

int32 encoder_get_right_total_count(void)
{
    return mock_right_count;
}

const line_control_output_t *line_control_get_output(void)
{
    return &mock_line;
}

motor_app_mode_enum motor_app_get_mode(void)
{
    return mock_motor_mode;
}

void motor_app_set_base_rpm(float base_rpm)
{
    mock_base_rpm = base_rpm;
}

void motor_app_set_line_follow_enabled(uint8 enabled)
{
    if (0u != enabled)
    {
        mock_motor_mode = MOTOR_APP_MODE_LINE_FOLLOW;
        mock_line_start_count++;
    }
    else
    {
        mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    }
}

void motor_app_stop(void)
{
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_stop_count++;
}

static void set_distance(float distance_m)
{
    float counts = distance_m * (float)ENCODER_COUNTS_PER_WHEEL_REV /
        (3.14159265f * CHASSIS_WHEEL_DIAMETER_M);

    mock_left_count = (int32)counts;
    mock_right_count = -(int32)counts;
}

static void reset_mocks(void)
{
    mock_now_ms = 0u;
    mock_grayscale_online = 1u;
    mock_left_count = 0;
    mock_right_count = 0;
    mock_line.line_lost = 0u;
    mock_line.marker_detected = 0u;
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
    mock_base_rpm = 0.0f;
    mock_line_start_count = 0u;
    mock_stop_count = 0u;
    no_load_lap_app_init();
}

static void test_start_allows_sensor_warmup_but_requires_idle_chassis(void)
{
    const no_load_lap_status_t *status;

    reset_mocks();
    mock_grayscale_online = 0u;
    assert(0u != no_load_lap_app_start());
    mock_now_ms = 1u;
    no_load_lap_app_process();
    status = no_load_lap_app_get_status();
    assert(status->state == NO_LOAD_LAP_RUNNING);
    mock_grayscale_online = 1u;
    mock_now_ms = 100u;
    no_load_lap_app_process();
    assert(status->state == NO_LOAD_LAP_RUNNING);

    reset_mocks();
    mock_motor_mode = MOTOR_APP_MODE_SPEED_TEST;
    assert(0u == no_load_lap_app_start());

    reset_mocks();
    assert(0u != no_load_lap_app_start());
    assert(mock_motor_mode == MOTOR_APP_MODE_LINE_FOLLOW);
    assert(mock_line_start_count == 1u);
    assert(fabsf(mock_base_rpm - NO_LOAD_LAP_CRUISE_RPM) < 0.001f);
}

static void test_start_marker_is_ignored_then_finish_stops(void)
{
    const no_load_lap_status_t *status;

    reset_mocks();
    mock_line.marker_detected = 1u;
    assert(0u != no_load_lap_app_start());
    mock_now_ms = 10u;
    no_load_lap_app_process();
    status = no_load_lap_app_get_status();
    assert(status->state == NO_LOAD_LAP_RUNNING);
    assert(0u == status->finish_armed);

    mock_line.marker_detected = 0u;
    set_distance(NO_LOAD_LAP_APPROACH_DISTANCE_M + 0.1f);
    mock_now_ms = 10000u;
    no_load_lap_app_process();
    assert(0u != status->finish_armed);
    assert(0u != status->approach_active);
    assert(fabsf(mock_base_rpm - NO_LOAD_LAP_APPROACH_RPM) < 0.001f);

    mock_line.marker_detected = 1u;
    mock_now_ms = 10010u;
    no_load_lap_app_process();
    assert(status->state == NO_LOAD_LAP_RUNNING);
    mock_now_ms = 10010u + NO_LOAD_LAP_MARKER_DEBOUNCE_MS;
    no_load_lap_app_process();
    assert(status->state == NO_LOAD_LAP_COMPLETE);
    assert(status->elapsed_ms == mock_now_ms);
    assert(mock_stop_count == 1u);
    assert(mock_motor_mode == MOTOR_APP_MODE_DISABLED);
}

static void test_timeout_line_loss_and_sensor_failure_stop(void)
{
    const no_load_lap_status_t *status;

    reset_mocks();
    assert(0u != no_load_lap_app_start());
    mock_now_ms = NO_LOAD_LAP_TIMEOUT_MS;
    no_load_lap_app_process();
    status = no_load_lap_app_get_status();
    assert(status->state == NO_LOAD_LAP_TIMEOUT);
    assert(mock_stop_count == 1u);

    reset_mocks();
    assert(0u != no_load_lap_app_start());
    mock_line.line_lost = 1u;
    mock_now_ms = 1u;
    no_load_lap_app_process();
    mock_now_ms = 1u + NO_LOAD_LAP_LINE_LOSS_TIMEOUT_MS;
    no_load_lap_app_process();
    assert(status->state == NO_LOAD_LAP_LINE_LOST);
    assert(mock_stop_count == 1u);

    reset_mocks();
    assert(0u != no_load_lap_app_start());
    mock_grayscale_online = 0u;
    mock_now_ms = 1u;
    no_load_lap_app_process();
    assert(status->state == NO_LOAD_LAP_RUNNING);
    mock_now_ms = 1u + NO_LOAD_LAP_SENSOR_OFFLINE_TIMEOUT_MS;
    no_load_lap_app_process();
    assert(status->state == NO_LOAD_LAP_SENSOR_OFFLINE);
    assert(mock_stop_count == 1u);
}

static void test_user_stop(void)
{
    const no_load_lap_status_t *status;

    reset_mocks();
    assert(0u != no_load_lap_app_start());
    mock_now_ms = 1234u;
    no_load_lap_app_stop();
    status = no_load_lap_app_get_status();
    assert(status->state == NO_LOAD_LAP_USER_STOP);
    assert(status->elapsed_ms == 1234u);
    assert(mock_stop_count == 1u);
}

int main(void)
{
    test_start_allows_sensor_warmup_but_requires_idle_chassis();
    test_start_marker_is_ignored_then_finish_stops();
    test_timeout_line_loss_and_sensor_failure_stop();
    test_user_stop();
    puts("no-load lap app tests passed");
    return 0;
}
