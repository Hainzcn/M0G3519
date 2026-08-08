#ifndef AB_RUN_APP_H_
#define AB_RUN_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    AB_RUN_IDLE = 0,
    AB_RUN_RUNNING,
    AB_RUN_BRAKING,
    AB_RUN_COMPLETE,
    AB_RUN_USER_STOP,
    AB_RUN_TIMEOUT,
    AB_RUN_BALANCE_FAULT,
    AB_RUN_LINE_LOST,
    AB_RUN_IMU_LOST,
} ab_run_state_enum;

typedef struct
{
    ab_run_state_enum state;
    uint8 passed_b;
    uint8 error_requirement_met;
    uint8 imu_valid;
    uint32 elapsed_ms;
    uint32 imu_age_ms;
    float distance_m;
    float max_abs_error_m;
    float imu_accel_mps2;
    float feedforward_accel_mps2;
} ab_run_status_t;

void ab_run_app_init(void);
void ab_run_app_process(void);
uint8 ab_run_app_start(void);
void ab_run_app_stop(void);
uint8 ab_run_app_is_running(void);
const ab_run_status_t *ab_run_app_get_status(void);

#endif
