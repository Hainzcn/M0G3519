#ifndef NO_LOAD_LAP_APP_H_
#define NO_LOAD_LAP_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    NO_LOAD_LAP_IDLE = 0,
    NO_LOAD_LAP_RUNNING,
    NO_LOAD_LAP_COMPLETE,
    NO_LOAD_LAP_USER_STOP,
    NO_LOAD_LAP_TIMEOUT,
    NO_LOAD_LAP_LINE_LOST,
    NO_LOAD_LAP_SENSOR_OFFLINE,
    NO_LOAD_LAP_CHASSIS_STOPPED,
} no_load_lap_state_enum;

typedef struct
{
    no_load_lap_state_enum state;
    uint8 finish_armed;
    uint8 approach_active;
    uint32 elapsed_ms;
    float distance_m;
} no_load_lap_status_t;

void no_load_lap_app_init(void);
void no_load_lap_app_process(void);
uint8 no_load_lap_app_start(void);
void no_load_lap_app_stop(void);
uint8 no_load_lap_app_is_running(void);
const no_load_lap_status_t *no_load_lap_app_get_status(void);

#endif
