#ifndef EMM42_DEMO_APP_H_
#define EMM42_DEMO_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    EMM42_DEMO_WAIT_POWER = 0,
    EMM42_DEMO_WAIT_ZERO,
    EMM42_DEMO_WAIT_ENABLE,
    EMM42_DEMO_FORWARD,
    EMM42_DEMO_WAIT_FORWARD,
    EMM42_DEMO_REVERSE,
    EMM42_DEMO_WAIT_REVERSE,
    EMM42_DEMO_RETURN_ZERO,
    EMM42_DEMO_WAIT_RETURN_ZERO,
    EMM42_DEMO_DONE,
    EMM42_DEMO_ERROR,
} emm42_demo_state_enum;

void emm42_demo_app_init(void);
void emm42_demo_app_process(void);
emm42_demo_state_enum emm42_demo_app_get_state(void);

#endif
