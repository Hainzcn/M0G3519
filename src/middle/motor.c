#include "motor.h"

#include "control_config.h"
#include "encoder.h"
#include "heartbeat.h"
#include "motor_hw.h"

static volatile uint32 motor_watchdog_last_kick_ms;
static volatile uint8  motor_watchdog_armed;

static int32 motor_clamp_speed(int32 speed)
{
    if (speed > (int32)MOTOR_SPEED_MAX)
    {
        speed = (int32)MOTOR_SPEED_MAX;
    }
    else if (speed < -(int32)MOTOR_SPEED_MAX)
    {
        speed = -(int32)MOTOR_SPEED_MAX;
    }
    return speed;
}

static int32 motor_apply_polarity(int32 speed, int32 polarity)
{
    return (polarity < 0) ? -speed : speed;
}

static void motor_watchdog_set_armed(uint8 armed)
{
    motor_watchdog_armed = armed;
}

void motor_watchdog_kick(void)
{
    motor_watchdog_last_kick_ms = heartbeat_get_ms();
}

void motor_watchdog_check(void)
{
    uint32 now_ms;
    uint32 elapsed_ms;

    if (0u == motor_watchdog_armed)
    {
        return;
    }

    now_ms     = heartbeat_get_ms();
    elapsed_ms = now_ms - motor_watchdog_last_kick_ms;
    if (MOTOR_WATCHDOG_TIMEOUT_MS < elapsed_ms)
    {
        motor_stop();
    }
}

void motor_init(void)
{
    motor_hw_init();
    encoder_init();
    motor_watchdog_last_kick_ms = 0;
    motor_watchdog_armed        = 0u;
    motor_stop();
}

void motor_set_speed(int32 left_speed, int32 right_speed)
{
    int32 left_output;
    int32 right_output;

    left_speed  = motor_clamp_speed(left_speed);
    right_speed = motor_clamp_speed(right_speed);
    left_output = motor_apply_polarity(left_speed,
                                       MOTOR_LEFT_OUTPUT_POLARITY);
    right_output = motor_apply_polarity(right_speed,
                                        MOTOR_RIGHT_OUTPUT_POLARITY);

    if (0u != MOTOR_OUTPUT_SWAP_LEFT_RIGHT)
    {
        motor_hw_set_duty(MOTOR_HW_LEFT, right_output);
        motor_hw_set_duty(MOTOR_HW_RIGHT, left_output);
    }
    else
    {
        motor_hw_set_duty(MOTOR_HW_LEFT, left_output);
        motor_hw_set_duty(MOTOR_HW_RIGHT, right_output);
    }

    if ((0 != left_output) || (0 != right_output))
    {
        motor_watchdog_set_armed(1u);
    }
    else
    {
        motor_watchdog_set_armed(0u);
    }
}

void motor_brake(void)
{
    motor_hw_brake(MOTOR_HW_LEFT);
    motor_hw_brake(MOTOR_HW_RIGHT);
    motor_watchdog_set_armed(1u);
}

void motor_stop(void)
{
    motor_hw_stop_all();
    motor_watchdog_set_armed(0u);
}
