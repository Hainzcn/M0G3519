#include "motor.h"
#include "encoder.h"
#include "motor_hw.h"

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

void motor_init(void)
{
    motor_hw_init();
    encoder_init();
    motor_stop();
}

void motor_set_speed(int32 left_speed, int32 right_speed)
{
    motor_hw_set_duty(MOTOR_HW_LEFT, motor_clamp_speed(left_speed));
    motor_hw_set_duty(MOTOR_HW_RIGHT, motor_clamp_speed(right_speed));
}

void motor_brake(void)
{
    motor_hw_brake(MOTOR_HW_LEFT);
    motor_hw_brake(MOTOR_HW_RIGHT);
}

void motor_stop(void)
{
    motor_hw_stop_all();
}
