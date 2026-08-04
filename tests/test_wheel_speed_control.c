#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "wheel_speed_control.h"

static int32 mock_left_rpm;
static int32 mock_right_rpm;

void encoder_update_speed(uint32 period_ms)
{
    (void)period_ms;
}

int32 encoder_get_left_rpm(void)
{
    return mock_left_rpm;
}

int32 encoder_get_right_rpm(void)
{
    return mock_right_rpm;
}

void motor_set_speed(int32 left_speed, int32 right_speed)
{
    (void)left_speed;
    (void)right_speed;
}

void motor_stop(void)
{
}

int main(void)
{
    const wheel_speed_control_status_t *status;
    float expected_speed;
    float expected_accel;
    uint32 index;

    wheel_speed_control_init();
    mock_left_rpm = 0;
    mock_right_rpm = 0;
    wheel_speed_control_update(10u, 0u);
    status = wheel_speed_control_get_status();
    assert(status->kinematics_valid != 0u);
    assert(status->measured_speed_mps == 0.0f);
    assert(status->measured_accel_mps2 == 0.0f);

    mock_left_rpm = 10;
    mock_right_rpm = -10;
    wheel_speed_control_update(10u, 0u);
    expected_speed = 10.0f * 3.14159265f * CHASSIS_WHEEL_DIAMETER_M / 60.0f;
    expected_accel = WHEEL_ACCEL_FILTER_ALPHA * expected_speed / 0.010f;
    assert(fabsf(status->measured_speed_mps - expected_speed) < 0.0001f);
    assert(fabsf(status->measured_accel_mps2 - expected_accel) < 0.0001f);

    wheel_speed_control_update(10u, 0u);
    expected_accel *= (1.0f - WHEEL_ACCEL_FILTER_ALPHA);
    assert(fabsf(status->measured_accel_mps2 - expected_accel) < 0.0001f);

    mock_left_rpm = 0;
    mock_right_rpm = 0;
    wheel_speed_control_update(10u, 0u);
    assert(status->measured_accel_mps2 < 0.0f);

    wheel_speed_control_reset();
    mock_left_rpm = 10;
    mock_right_rpm = 10;
    wheel_speed_control_update(10u, 0u);
    assert(fabsf(status->measured_speed_mps) < 0.0001f);

    wheel_speed_control_reset();
    mock_left_rpm = 0;
    mock_right_rpm = 0;
    wheel_speed_control_set_target(100.0f, 100.0f);
    for (index = 0u; index < 10u; index++)
    {
        wheel_speed_control_update(10u, 1u);
    }
    assert(status->left_duty >= 2900);

    mock_left_rpm = 100;
    mock_right_rpm = -100;
    wheel_speed_control_set_rapid_brake_enabled(1u);
    wheel_speed_control_set_target(0.0f, 0.0f);
    wheel_speed_control_update(10u, 1u);
    assert(status->left_duty <= 1500);
    assert(status->left_duty < 2900);

    puts("wheel speed control kinematics tests passed");
    return 0;
}
