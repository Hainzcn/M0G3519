#ifndef STOP_TEST_APP_H_
#define STOP_TEST_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    STOP_TEST_IDLE = 0,
    STOP_TEST_WAIT_BALANCE,
    STOP_TEST_TO_POSITIVE,
    STOP_TEST_SETTLE_POSITIVE,
    STOP_TEST_TO_NEGATIVE,
    STOP_TEST_SETTLE_NEGATIVE,
    STOP_TEST_COMPLETE,
    STOP_TEST_RETURN_CENTER,
    STOP_TEST_TIMEOUT,
    STOP_TEST_FAULT,
    STOP_TEST_USER_STOP,
} stop_test_state_enum;

typedef enum
{
    STOP_TEST_STOP_NONE = 0,
    STOP_TEST_STOP_COMPLETE,
    STOP_TEST_STOP_USER,
    STOP_TEST_STOP_TIMEOUT,
    STOP_TEST_STOP_BALANCE,
    STOP_TEST_STOP_TARGET_REJECTED,
    STOP_TEST_STOP_START_REJECTED,
} stop_test_stop_reason_enum;

typedef struct
{
    stop_test_state_enum state;
    stop_test_stop_reason_enum stop_reason;
    uint32 elapsed_ms;
    float target_position_m;
    float positive_max_abs_error_m;
    float negative_max_abs_error_m;
    float max_abs_error_m;
    uint8 positive_reached;
    uint8 negative_reached;
    uint8 time_requirement_met;
    uint8 error_requirement_met;
    uint8 overall_requirement_met;
} stop_test_status_t;

void stop_test_app_init(void);
void stop_test_app_process(void);
uint8 stop_test_app_start(void);
void stop_test_app_stop(void);
uint8 stop_test_app_is_running(void);
void stop_test_app_set_positive_target_m(float target_m);
float stop_test_app_get_positive_target_m(void);
void stop_test_app_set_negative_target_m(float target_m);
float stop_test_app_get_negative_target_m(void);
const stop_test_status_t *stop_test_app_get_status(void);

#endif
