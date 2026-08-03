#ifndef DRIVE_BALANCE_DEMO_APP_H_
#define DRIVE_BALANCE_DEMO_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    DRIVE_BALANCE_DEMO_IDLE = 0,
    DRIVE_BALANCE_DEMO_RUNNING_CENTER,
    DRIVE_BALANCE_DEMO_RUNNING_CAPTURED,
    DRIVE_BALANCE_DEMO_COMPLETE,
    DRIVE_BALANCE_DEMO_ABORTED,
    DRIVE_BALANCE_DEMO_TIMEOUT,
    DRIVE_BALANCE_DEMO_FAULT_STOP,
} drive_balance_demo_state_enum;

typedef enum
{
    DRIVE_BALANCE_STOP_NONE = 0,
    DRIVE_BALANCE_STOP_LAP_COMPLETE,
    DRIVE_BALANCE_STOP_USER,
    DRIVE_BALANCE_STOP_TIMEOUT,
    DRIVE_BALANCE_STOP_BALANCE,
    DRIVE_BALANCE_STOP_LINE,
    DRIVE_BALANCE_STOP_IMU,
    DRIVE_BALANCE_STOP_START_REJECTED,
} drive_balance_demo_stop_reason_enum;

typedef struct
{
    drive_balance_demo_state_enum state;
    drive_balance_demo_stop_reason_enum stop_reason;
    uint8 finish_armed;
    uint32 elapsed_ms;
    float target_position_m;
    float max_abs_error_m;
    float distance_m;
} drive_balance_demo_status_t;

void drive_balance_demo_app_init(void);
void drive_balance_demo_app_process(void);
uint8 drive_balance_demo_app_is_running(void);
const drive_balance_demo_status_t *drive_balance_demo_app_get_status(void);

#endif
