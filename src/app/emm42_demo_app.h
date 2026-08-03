#ifndef EMM42_DEMO_APP_H_
#define EMM42_DEMO_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    EMM42_DEMO_WAIT_POWER = 0,
    EMM42_DEMO_WAIT_ZERO,
    EMM42_DEMO_WAIT_ENABLE,
    EMM42_DEMO_MOVE_ANGLE,
    EMM42_DEMO_WAIT_ANGLE,
    EMM42_DEMO_READY,
    EMM42_DEMO_RECORDING,
    EMM42_DEMO_DONE,
    EMM42_DEMO_ERROR,
} emm42_demo_state_enum;

void emm42_demo_app_init(void);
void emm42_demo_app_process(void);
emm42_demo_state_enum emm42_demo_app_get_state(void);
uint16 emm42_demo_app_get_trial_id(void);
float emm42_demo_app_get_target_lever_deg(void);
float emm42_demo_app_get_target_motor_deg(void);
float emm42_demo_app_get_motor_feedback_deg(void);
uint8 emm42_demo_app_is_motor_feedback_valid(void);
uint8 emm42_demo_app_is_recording(void);
uint8 emm42_demo_app_is_active(void);

#endif
