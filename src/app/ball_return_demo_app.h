#ifndef BALL_RETURN_DEMO_APP_H_
#define BALL_RETURN_DEMO_APP_H_

#include "zf_common_typedef.h"
#include "ball_motion_profile.h"

typedef enum
{
    BALL_RETURN_DEMO_WAIT_POWER = 0,
    BALL_RETURN_DEMO_WAIT_ZERO,
    BALL_RETURN_DEMO_WAIT_ENABLE,
    BALL_RETURN_DEMO_MOVE_LEVEL,
    BALL_RETURN_DEMO_WAIT_LEVEL,
    BALL_RETURN_DEMO_READY,
    BALL_RETURN_DEMO_RUNNING,
    BALL_RETURN_DEMO_SETTLING,
    BALL_RETURN_DEMO_DONE,
    BALL_RETURN_DEMO_ERROR,
} ball_return_demo_state_enum;

void ball_return_demo_app_init(void);
void ball_return_demo_app_process(void);
ball_return_demo_state_enum ball_return_demo_app_get_state(void);
float ball_return_demo_app_get_reference_position_m(void);
float ball_return_demo_app_get_reference_velocity_mps(void);
float ball_return_demo_app_get_raw_lever_angle_deg(void);
float ball_return_demo_app_get_lever_angle_deg(void);
float ball_return_demo_app_get_motor_target_deg(void);
ball_motion_phase_enum ball_return_demo_app_get_motion_phase(void);

#endif
