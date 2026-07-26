#include "ti_msp_dl_config.h"

#include "motor_app.h"
#include "motor.h"

#define MOTOR_APP_DEMO_STEP_DELAY_CYCLES   (80000000 / 2)     // 约 0.5s @ 80MHz 系统时钟，仅供 motor_app_demo 使用

void motor_app_init(void)
{
    motor_init();       // motor_init 内部已调用 motor_stop，此处保持默认静止状态
}

void motor_app_demo(void)
{
    motor_set_speed(MOTOR_SPEED_MAX / 2, MOTOR_SPEED_MAX / 2);
    delay_cycles(MOTOR_APP_DEMO_STEP_DELAY_CYCLES);

    motor_set_speed(-(MOTOR_SPEED_MAX / 2), -(MOTOR_SPEED_MAX / 2));
    delay_cycles(MOTOR_APP_DEMO_STEP_DELAY_CYCLES);

    motor_stop();
}
