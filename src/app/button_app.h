#ifndef BUTTON_APP_H_
#define BUTTON_APP_H_

#include "zf_common_typedef.h"

#define BUTTON_APP_TUNING_LONG_PRESS_MS    (1000u)

typedef enum
{
    BUTTON_APP_MODE_NO_LOAD = 0,
    BUTTON_APP_MODE_STOP_TEST,
    BUTTON_APP_MODE_AB,
    BUTTON_APP_MODE_BALL_LAP,
    BUTTON_APP_MODE_ARBITRARY,
} button_app_mode_enum;

void button_app_init(void);
void button_app_process(void);
button_app_mode_enum button_app_get_selected_mode(void);
uint8 button_app_is_running(void);

#endif
