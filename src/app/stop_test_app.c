#include "stop_test_app.h"

#include <stdio.h>

#include "control_config.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "motor_app.h"
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
#include "balance_simple_app.h"
#else
#include "balance_app.h"
#endif

typedef struct
{
    float position_m;
    float velocity_mps;
    uint8 ready;
    uint8 failed;
} stop_test_balance_snapshot_t;

static stop_test_status_t stop_test_status;
static uint32 stop_test_ready_start_ms;
static uint32 stop_test_motion_start_ms;
static uint32 stop_test_settle_start_ms;
static uint32 stop_test_center_retry_ms;
static uint8 stop_test_center_target_accepted;

static float stop_test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 stop_test_running(void)
{
    return ((STOP_TEST_WAIT_BALANCE == stop_test_status.state) ||
            (STOP_TEST_TO_POSITIVE == stop_test_status.state) ||
            (STOP_TEST_SETTLE_POSITIVE == stop_test_status.state) ||
            (STOP_TEST_TO_NEGATIVE == stop_test_status.state) ||
            (STOP_TEST_SETTLE_NEGATIVE == stop_test_status.state)) ? 1u : 0u;
}

static void stop_test_read_balance(stop_test_balance_snapshot_t *snapshot)
{
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    const balance_simple_status_t *balance = balance_simple_app_get_status();
    uint16 required_flags = BALANCE_SIMPLE_FLAG_OBSERVER_VALID |
                            BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;

    snapshot->position_m = balance->estimated_position_m;
    snapshot->velocity_mps = balance->estimated_velocity_mps;
    snapshot->ready = ((((BALANCE_SIMPLE_ACTIVE == balance->state) ||
                         (BALANCE_SIMPLE_STATIC_LOCK == balance->state)) &&
                        ((balance->flags & required_flags) == required_flags))) ?
        1u : 0u;
    snapshot->failed = ((BALANCE_SIMPLE_FAULT == balance->state) ||
                        (BALANCE_SIMPLE_SAFE_RETURN == balance->state) ||
                        (BALANCE_SIMPLE_DISABLED == balance->state)) ? 1u : 0u;
#else
    const balance_app_status_t *balance = balance_app_get_status();

    snapshot->position_m = balance->estimated_position_m;
    snapshot->velocity_mps = balance->estimated_velocity_mps;
    snapshot->ready = ((BALANCE_APP_ACTIVE == balance->state) &&
        (0u == (balance->flags & BALANCE_APP_FLAG_SEQUENCE_ACTIVE))) ? 1u : 0u;
    snapshot->failed = ((BALANCE_APP_FAULT == balance->state) ||
                        (BALANCE_APP_UNCONFIGURED == balance->state)) ? 1u : 0u;
#endif
}

static uint8 stop_test_prepare_balance(void)
{
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    return balance_simple_app_start();
#else
    const balance_app_status_t *balance = balance_app_get_status();

    if ((BALANCE_APP_FAULT == balance->state) ||
        (BALANCE_APP_UNCONFIGURED == balance->state))
    {
        return 0u;
    }
    balance_app_cancel_motion();
    return 1u;
#endif
}

static uint8 stop_test_set_target(float target_position_m)
{
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    return balance_simple_app_set_target_position_m(target_position_m);
#else
    return balance_app_set_target_position_m(target_position_m);
#endif
}

static void stop_test_return_to_center(void)
{
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    (void)balance_simple_app_set_target_position_m(0.0f);
#else
    balance_app_cancel_motion();
#endif
}

static void stop_test_log_result(void)
{
    char message[176];

    snprintf(message, sizeof(message),
        "[stop-test] end=%u,t=%lu,poserr=%.3f,negerr=%.3f,ok=%u\r\n",
        (unsigned int)stop_test_status.stop_reason,
        (unsigned long)stop_test_status.elapsed_ms,
        (double)stop_test_status.positive_max_abs_error_m,
        (double)stop_test_status.negative_max_abs_error_m,
        (unsigned int)stop_test_status.overall_requirement_met);
    heartbeat_hw_uart_send_string(message);
}

static void stop_test_finish(stop_test_state_enum state,
                             stop_test_stop_reason_enum reason,
                             uint8 return_to_center)
{
    stop_test_status.state = state;
    stop_test_status.stop_reason = reason;
    stop_test_status.max_abs_error_m =
        (stop_test_status.positive_max_abs_error_m >
         stop_test_status.negative_max_abs_error_m) ?
        stop_test_status.positive_max_abs_error_m :
        stop_test_status.negative_max_abs_error_m;
    stop_test_status.time_requirement_met =
        ((STOP_TEST_STOP_COMPLETE == reason) &&
         (stop_test_status.elapsed_ms <= STOP_TEST_TIMEOUT_MS)) ? 1u : 0u;
    stop_test_status.error_requirement_met =
        ((0u != stop_test_status.positive_reached) &&
         (0u != stop_test_status.negative_reached) &&
         (stop_test_status.positive_max_abs_error_m <=
          STOP_TEST_MAX_ENDPOINT_ERROR_M) &&
         (stop_test_status.negative_max_abs_error_m <=
          STOP_TEST_MAX_ENDPOINT_ERROR_M)) ? 1u : 0u;
    stop_test_status.overall_requirement_met =
        ((0u != stop_test_status.time_requirement_met) &&
         (0u != stop_test_status.error_requirement_met)) ? 1u : 0u;
    if (0u != return_to_center)
    {
        stop_test_return_to_center();
        stop_test_status.target_position_m = 0.0f;
    }
    stop_test_log_result();
}

static uint8 stop_test_begin_motion(uint32 now_ms)
{
    if (0u == stop_test_set_target(STOP_TEST_POSITIVE_TARGET_M))
    {
        stop_test_finish(STOP_TEST_FAULT,
                         STOP_TEST_STOP_TARGET_REJECTED, 1u);
        return 0u;
    }
    stop_test_motion_start_ms = now_ms;
    stop_test_status.elapsed_ms = 0u;
    stop_test_status.target_position_m = STOP_TEST_POSITIVE_TARGET_M;
    stop_test_status.state = STOP_TEST_TO_POSITIVE;
    heartbeat_hw_uart_send_string("[stop-test] motion start\r\n");
    return 1u;
}

static uint8 stop_test_arrived(const stop_test_balance_snapshot_t *balance,
                               float target_position_m)
{
    return ((stop_test_abs(target_position_m - balance->position_m) <=
             STOP_TEST_ARRIVAL_TOLERANCE_M) &&
            (stop_test_abs(balance->velocity_mps) <=
             STOP_TEST_VELOCITY_TOLERANCE_MPS)) ? 1u : 0u;
}

static void stop_test_process_center_return(uint32 now_ms)
{
    stop_test_balance_snapshot_t balance;

    stop_test_read_balance(&balance);
    if (0u != balance.failed)
    {
        stop_test_status.state = STOP_TEST_FAULT;
        stop_test_status.stop_reason = STOP_TEST_STOP_BALANCE;
        heartbeat_hw_uart_send_string(
            "[stop-test] center return stopped: balance fault\r\n");
        return;
    }
    if ((0u == stop_test_center_target_accepted) &&
        ((now_ms - stop_test_center_retry_ms) >=
         STOP_TEST_CENTER_RETRY_MS))
    {
        stop_test_center_retry_ms = now_ms;
        if (0u != stop_test_set_target(0.0f))
        {
            stop_test_center_target_accepted = 1u;
            stop_test_status.target_position_m = 0.0f;
            heartbeat_hw_uart_send_string(
                "[stop-test] center target accepted\r\n");
        }
    }
    if ((0u != stop_test_center_target_accepted) &&
        (0u != balance.ready) &&
        (stop_test_abs(balance.position_m) <=
         STOP_TEST_CENTER_TOLERANCE_M) &&
        (stop_test_abs(balance.velocity_mps) <=
         STOP_TEST_VELOCITY_TOLERANCE_MPS))
    {
        stop_test_status.state = STOP_TEST_IDLE;
        heartbeat_hw_uart_send_string("[stop-test] centered\r\n");
    }
}

static void stop_test_update_positive(
    const stop_test_balance_snapshot_t *balance, uint32 now_ms)
{
    float error = stop_test_abs(STOP_TEST_POSITIVE_TARGET_M -
                                balance->position_m);

    if (STOP_TEST_TO_POSITIVE == stop_test_status.state)
    {
        if (0u != stop_test_arrived(balance, STOP_TEST_POSITIVE_TARGET_M))
        {
            stop_test_status.positive_max_abs_error_m = error;
            stop_test_settle_start_ms = now_ms;
            stop_test_status.state = STOP_TEST_SETTLE_POSITIVE;
        }
        return;
    }
    if (0u == stop_test_arrived(balance, STOP_TEST_POSITIVE_TARGET_M))
    {
        stop_test_status.state = STOP_TEST_TO_POSITIVE;
        return;
    }
    if (error > stop_test_status.positive_max_abs_error_m)
    {
        stop_test_status.positive_max_abs_error_m = error;
    }
    if ((now_ms - stop_test_settle_start_ms) < STOP_TEST_ENDPOINT_SETTLE_MS)
    {
        return;
    }
    stop_test_status.positive_reached = 1u;
    if (0u == stop_test_set_target(STOP_TEST_NEGATIVE_TARGET_M))
    {
        stop_test_finish(STOP_TEST_FAULT,
                         STOP_TEST_STOP_TARGET_REJECTED, 1u);
        return;
    }
    stop_test_status.target_position_m = STOP_TEST_NEGATIVE_TARGET_M;
    stop_test_status.state = STOP_TEST_TO_NEGATIVE;
    heartbeat_hw_uart_send_string("[stop-test] positive reached; return\r\n");
}

static void stop_test_update_negative(
    const stop_test_balance_snapshot_t *balance, uint32 now_ms)
{
    float error = stop_test_abs(STOP_TEST_NEGATIVE_TARGET_M -
                                balance->position_m);

    if (STOP_TEST_TO_NEGATIVE == stop_test_status.state)
    {
        if (0u != stop_test_arrived(balance, STOP_TEST_NEGATIVE_TARGET_M))
        {
            stop_test_status.negative_max_abs_error_m = error;
            stop_test_settle_start_ms = now_ms;
            stop_test_status.state = STOP_TEST_SETTLE_NEGATIVE;
        }
        return;
    }
    if (0u == stop_test_arrived(balance, STOP_TEST_NEGATIVE_TARGET_M))
    {
        stop_test_status.state = STOP_TEST_TO_NEGATIVE;
        return;
    }
    if (error > stop_test_status.negative_max_abs_error_m)
    {
        stop_test_status.negative_max_abs_error_m = error;
    }
    if ((now_ms - stop_test_settle_start_ms) < STOP_TEST_ENDPOINT_SETTLE_MS)
    {
        return;
    }
    stop_test_status.negative_reached = 1u;
    stop_test_status.elapsed_ms = now_ms - stop_test_motion_start_ms;
    stop_test_finish(STOP_TEST_COMPLETE, STOP_TEST_STOP_COMPLETE, 0u);
}

void stop_test_app_init(void)
{
    stop_test_status.state = STOP_TEST_IDLE;
    stop_test_status.stop_reason = STOP_TEST_STOP_NONE;
    stop_test_status.elapsed_ms = 0u;
    stop_test_status.target_position_m = 0.0f;
    stop_test_status.positive_max_abs_error_m = 0.0f;
    stop_test_status.negative_max_abs_error_m = 0.0f;
    stop_test_status.max_abs_error_m = 0.0f;
    stop_test_status.positive_reached = 0u;
    stop_test_status.negative_reached = 0u;
    stop_test_status.time_requirement_met = 0u;
    stop_test_status.error_requirement_met = 0u;
    stop_test_status.overall_requirement_met = 0u;
    stop_test_ready_start_ms = 0u;
    stop_test_motion_start_ms = 0u;
    stop_test_settle_start_ms = 0u;
    stop_test_center_retry_ms = 0u;
    stop_test_center_target_accepted = 0u;
}

uint8 stop_test_app_start(void)
{
    stop_test_balance_snapshot_t balance;
    uint32 now_ms = heartbeat_get_ms();

    motor_app_stop();
    stop_test_app_init();
    if (0u == stop_test_prepare_balance())
    {
        stop_test_status.stop_reason = STOP_TEST_STOP_START_REJECTED;
        heartbeat_hw_uart_send_string("[stop-test] start rejected\r\n");
        return 0u;
    }
    stop_test_status.state = STOP_TEST_WAIT_BALANCE;
    stop_test_ready_start_ms = now_ms;
    stop_test_read_balance(&balance);
    if (0u != balance.failed)
    {
        stop_test_status.state = STOP_TEST_IDLE;
        stop_test_status.stop_reason = STOP_TEST_STOP_START_REJECTED;
        return 0u;
    }
    if (0u != balance.ready)
    {
        return stop_test_begin_motion(now_ms);
    }
    heartbeat_hw_uart_send_string("[stop-test] waiting for balance\r\n");
    return 1u;
}

void stop_test_app_process(void)
{
    stop_test_balance_snapshot_t balance;
    uint32 now_ms;

    now_ms = heartbeat_get_ms();
    if (STOP_TEST_RETURN_CENTER == stop_test_status.state)
    {
        stop_test_process_center_return(now_ms);
        return;
    }
    if (0u == stop_test_running())
    {
        return;
    }
    stop_test_read_balance(&balance);
    if (STOP_TEST_WAIT_BALANCE == stop_test_status.state)
    {
        if (0u != balance.failed)
        {
            stop_test_finish(STOP_TEST_FAULT,
                             STOP_TEST_STOP_BALANCE, 0u);
        }
        else if (0u != balance.ready)
        {
            (void)stop_test_begin_motion(now_ms);
        }
        else if ((now_ms - stop_test_ready_start_ms) >
                 STOP_TEST_READY_TIMEOUT_MS)
        {
            stop_test_finish(STOP_TEST_FAULT,
                             STOP_TEST_STOP_BALANCE, 0u);
        }
        return;
    }

    stop_test_status.elapsed_ms = now_ms - stop_test_motion_start_ms;
    if (0u != balance.failed)
    {
        stop_test_finish(STOP_TEST_FAULT, STOP_TEST_STOP_BALANCE, 0u);
        return;
    }
    if (stop_test_status.elapsed_ms > STOP_TEST_TIMEOUT_MS)
    {
        stop_test_finish(STOP_TEST_TIMEOUT,
                         STOP_TEST_STOP_TIMEOUT, 1u);
        return;
    }
    if (0u != balance.ready)
    {
        if ((STOP_TEST_TO_POSITIVE == stop_test_status.state) ||
            (STOP_TEST_SETTLE_POSITIVE == stop_test_status.state))
        {
            stop_test_update_positive(&balance, now_ms);
        }
        else
        {
            stop_test_update_negative(&balance, now_ms);
        }
    }
}

void stop_test_app_stop(void)
{
    if (0u == stop_test_running())
    {
        if ((STOP_TEST_COMPLETE == stop_test_status.state) &&
            (0.0f != stop_test_status.target_position_m))
        {
            stop_test_status.state = STOP_TEST_RETURN_CENTER;
            stop_test_center_target_accepted =
                stop_test_set_target(0.0f);
            stop_test_center_retry_ms = heartbeat_get_ms();
            if (0u != stop_test_center_target_accepted)
            {
                stop_test_status.target_position_m = 0.0f;
                heartbeat_hw_uart_send_string(
                    "[stop-test] result acknowledged; center target accepted\r\n");
            }
            else
            {
                heartbeat_hw_uart_send_string(
                    "[stop-test] result acknowledged; center target pending\r\n");
            }
        }
        return;
    }
    if (STOP_TEST_WAIT_BALANCE != stop_test_status.state)
    {
        stop_test_status.elapsed_ms =
            heartbeat_get_ms() - stop_test_motion_start_ms;
    }
    stop_test_finish(STOP_TEST_USER_STOP, STOP_TEST_STOP_USER, 1u);
}

uint8 stop_test_app_is_running(void)
{
    return stop_test_running();
}

const stop_test_status_t *stop_test_app_get_status(void)
{
    return &stop_test_status;
}
