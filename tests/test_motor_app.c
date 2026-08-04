#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "grayscale.h"
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"

static uint32 mock_now_ms;
static uint32 mock_scan_sequence;
static uint8 mock_grayscale_values[GRAYSCALE_CHANNELS];
static line_control_output_t mock_line;
static wheel_speed_control_status_t mock_wheel;
static uint32 mock_motor_brake_count;
static uint32 mock_motor_stop_count;
static uint32 mock_wheel_update_count;
static uint8 mock_wheel_update_enabled;

uint32 heartbeat_get_ms(void) { return mock_now_ms; }
void heartbeat_hw_uart_send_string(const char *message) { (void)message; }

void motor_init(void) {}
void motor_brake(void) { mock_motor_brake_count++; }
void motor_stop(void) { mock_motor_stop_count++; }

uint32 grayscale_get_scan_sequence(void) { return mock_scan_sequence; }
const uint8 *grayscale_get_values(void) { return mock_grayscale_values; }

void line_control_init(void) { memset(&mock_line, 0, sizeof(mock_line)); }
void line_control_reset(void) { memset(&mock_line, 0, sizeof(mock_line)); }
void line_control_set_base_rpm(float base_rpm) { (void)base_rpm; }
void line_control_set_base_rpm_immediate(float base_rpm) { (void)base_rpm; }
float line_control_get_base_rpm(void) { return 0.0f; }
void line_control_update(const uint8 values[GRAYSCALE_CHANNELS],
                         uint32 now_ms, float dt_s)
{
    (void)values;
    (void)now_ms;
    (void)dt_s;
}
const line_control_output_t *line_control_get_output(void) { return &mock_line; }

void wheel_speed_control_init(void)
{
    memset(&mock_wheel, 0, sizeof(mock_wheel));
}
void wheel_speed_control_reset(void) {}
void wheel_speed_control_set_target(float left_rpm, float right_rpm)
{
    (void)left_rpm;
    (void)right_rpm;
}
void wheel_speed_control_set_rapid_brake_enabled(uint8 enabled)
{
    (void)enabled;
}
void wheel_speed_control_update(uint32 period_ms, uint8 enabled)
{
    (void)period_ms;
    mock_wheel_update_count++;
    mock_wheel_update_enabled = enabled;
}
const wheel_speed_control_status_t *wheel_speed_control_get_status(void)
{
    return &mock_wheel;
}

static void test_brake_persists_until_explicit_stop(void)
{
    mock_now_ms = 0u;
    mock_scan_sequence = 0u;
    mock_motor_brake_count = 0u;
    mock_motor_stop_count = 0u;
    mock_wheel_update_count = 0u;
    mock_wheel_update_enabled = 1u;
    motor_app_init();

    motor_app_brake();
    assert(MOTOR_APP_MODE_DISABLED == motor_app_get_mode());
    assert(mock_motor_brake_count == 1u);

    mock_now_ms = 10u;
    motor_app_process();
    assert(mock_motor_brake_count == 2u);
    assert(mock_wheel_update_count == 0u);

    motor_app_stop();
    assert(mock_motor_stop_count == 1u);
    mock_now_ms = 20u;
    motor_app_process();
    assert(mock_motor_brake_count == 2u);
    assert(mock_wheel_update_count == 1u);
    assert(0u == mock_wheel_update_enabled);
}

static void test_starting_line_follow_releases_brake(void)
{
    motor_app_brake();
    motor_app_set_line_follow_enabled(1u);
    assert(MOTOR_APP_MODE_LINE_FOLLOW == motor_app_get_mode());

    mock_scan_sequence++;
    mock_now_ms += 10u;
    motor_app_process();
    assert(mock_motor_brake_count == 3u);
}

int main(void)
{
    test_brake_persists_until_explicit_stop();
    test_starting_line_follow_releases_brake();
    puts("motor app brake tests passed");
    return 0;
}
