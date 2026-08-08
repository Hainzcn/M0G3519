#ifndef WHEEL_SPEED_CONTROL_H_
#define WHEEL_SPEED_CONTROL_H_

#include "zf_common_typedef.h"

typedef struct
{
    float left_target_rpm;
    float right_target_rpm;
    float left_measured_rpm;
    float right_measured_rpm;
    float planned_speed_mps;
    float planned_accel_mps2;
    float measured_speed_mps;
    float measured_accel_mps2;
    float left_feedforward_pwm;
    float right_feedforward_pwm;
    float left_feedback_pwm;
    float right_feedback_pwm;
    int32 left_duty;
    int32 right_duty;
    uint8 kinematics_valid;
    uint8 left_saturated;
    uint8 right_saturated;
} wheel_speed_control_status_t;

void wheel_speed_control_init(void);
void wheel_speed_control_reset(void);
void wheel_speed_control_set_target(float left_rpm, float right_rpm);
void wheel_speed_control_set_rapid_brake_enabled(uint8 enabled);
void wheel_speed_control_update(uint32 period_ms, uint8 enabled);
const wheel_speed_control_status_t *wheel_speed_control_get_status(void);

#endif
