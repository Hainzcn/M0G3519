#ifndef MOTOR_APP_H_
#define MOTOR_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    MOTOR_APP_MODE_DISABLED = 0,
    MOTOR_APP_MODE_SPEED_TEST,
    MOTOR_APP_MODE_RIGHT_CIRCLE_DEMO,
    MOTOR_APP_MODE_LINE_FOLLOW,
} motor_app_mode_enum;

void motor_app_init(void);
void motor_app_process(void);
void motor_app_stop(void);
void motor_app_set_line_follow_enabled(uint8 enabled);
void motor_app_set_base_rpm(float base_rpm);
void motor_app_set_speed_test(float left_rpm, float right_rpm);
void motor_app_set_right_circle_demo(float center_rpm);
motor_app_mode_enum motor_app_get_mode(void);

#endif
