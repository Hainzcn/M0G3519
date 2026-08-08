#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "balance_simple_app.h"
#include "control_config.h"
#include "stop_test_app.h"

static uint32 mock_now_ms;
static balance_simple_status_t mock_balance;
static uint8 mock_balance_start_result;
static uint8 mock_target_result;
static float mock_targets[8];
static uint8 mock_target_count;
static uint32 mock_motor_stop_count;
static uint8 mock_stop_test_mode;
static uint32 mock_stop_test_mode_enable_count;
static uint32 mock_stop_test_mode_disable_count;
static uint8 mock_balance_start_resets_tuning;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

void motor_app_stop(void)
{
    mock_motor_stop_count++;
}

uint8 balance_simple_app_start(void)
{
    if (0u != mock_balance_start_resets_tuning)
    {
        mock_stop_test_mode = 0u;
    }
    return mock_balance_start_result;
}

void balance_simple_app_set_stop_test_mode(uint8 enabled)
{
    mock_stop_test_mode = (0u != enabled) ? 1u : 0u;
    if (0u != enabled)
    {
        mock_stop_test_mode_enable_count++;
    }
    else
    {
        mock_stop_test_mode_disable_count++;
    }
}

uint8 balance_simple_app_set_target_position_m(float target_position_m)
{
    if ((0u != mock_target_result) &&
        (mock_target_count < (uint8)(sizeof(mock_targets) /
                                     sizeof(mock_targets[0]))))
    {
        mock_targets[mock_target_count++] = target_position_m;
    }
    return mock_target_result;
}

const balance_simple_status_t *balance_simple_app_get_status(void)
{
    return &mock_balance;
}

static void reset_mocks(void)
{
    mock_now_ms = 0u;
    memset(&mock_balance, 0, sizeof(mock_balance));
    mock_balance.state = BALANCE_SIMPLE_ACTIVE;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_OBSERVER_VALID |
                         BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    mock_balance_start_result = 1u;
    mock_target_result = 1u;
    memset(mock_targets, 0, sizeof(mock_targets));
    mock_target_count = 0u;
    mock_motor_stop_count = 0u;
    mock_stop_test_mode = 0u;
    mock_stop_test_mode_enable_count = 0u;
    mock_stop_test_mode_disable_count = 0u;
    mock_balance_start_resets_tuning = 0u;
    stop_test_app_init();
}

static void test_start_enables_stop_test_tuning_after_balance_init_order(void)
{
    reset_mocks();
    mock_stop_test_mode = 1u;
    mock_balance_start_resets_tuning = 1u;

    assert(0u != stop_test_app_start());
    assert(mock_stop_test_mode != 0u);
    assert(mock_stop_test_mode_enable_count == 1u);
}

static void test_complete_plus_five_to_minus_five(void)
{
    const stop_test_status_t *status;

    reset_mocks();
    assert(0u != stop_test_app_start());
    assert(mock_motor_stop_count == 1u);
    assert(mock_stop_test_mode != 0u);
    assert(mock_stop_test_mode_enable_count == 1u);
    assert(mock_target_count == 1u);
    assert(fabsf(mock_targets[0] - STOP_TEST_POSITIVE_TARGET_M) < 0.0001f);

    mock_now_ms = 1000u;
    mock_balance.estimated_position_m =
        STOP_TEST_POSITIVE_TARGET_M - 0.002f;
    mock_balance.estimated_velocity_mps = 0.010f;
    stop_test_app_process();
    assert(stop_test_app_get_status()->state == STOP_TEST_SETTLE_POSITIVE);

    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    mock_balance.estimated_position_m =
        STOP_TEST_POSITIVE_TARGET_M + 0.002f;
    stop_test_app_process();
    assert(stop_test_app_get_status()->state == STOP_TEST_TO_NEGATIVE);
    assert(mock_target_count == 2u);
    assert(fabsf(mock_targets[1] - STOP_TEST_NEGATIVE_TARGET_M) < 0.0001f);

    mock_now_ms = 4000u;
    mock_balance.estimated_position_m =
        STOP_TEST_NEGATIVE_TARGET_M + 0.002f;
    mock_balance.estimated_velocity_mps = -0.010f;
    stop_test_app_process();
    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    mock_balance.estimated_position_m =
        STOP_TEST_NEGATIVE_TARGET_M - 0.002f;
    mock_balance.estimated_velocity_mps = 0.0f;
    stop_test_app_process();

    status = stop_test_app_get_status();
    assert(status->state == STOP_TEST_COMPLETE);
    assert(status->stop_reason == STOP_TEST_STOP_COMPLETE);
    assert(status->elapsed_ms == 4200u);
    assert(status->positive_reached != 0u);
    assert(status->negative_reached != 0u);
    assert(fabsf(status->positive_max_abs_error_m - 0.002f) < 0.0001f);
    assert(fabsf(status->negative_max_abs_error_m - 0.002f) < 0.0001f);
    assert(status->time_requirement_met != 0u);
    assert(status->error_requirement_met != 0u);
    assert(status->overall_requirement_met != 0u);
    assert(0u == stop_test_app_is_running());
    assert(mock_stop_test_mode != 0u);
    assert(mock_target_count == 2u);
    assert(fabsf(mock_targets[1] - STOP_TEST_NEGATIVE_TARGET_M) < 0.0001f);

    stop_test_app_stop();
    assert(status->state == STOP_TEST_RETURN_CENTER);
    assert(mock_target_count == 3u);
    assert(mock_targets[2] == 0.0f);
    assert(status->target_position_m == 0.0f);

    mock_now_ms += 100u;
    mock_balance.estimated_position_m = 0.002f;
    mock_balance.estimated_velocity_mps = 0.0f;
    stop_test_app_process();
    assert(status->state == STOP_TEST_IDLE);
    assert(mock_stop_test_mode == 0u);
    assert(mock_stop_test_mode_disable_count == 1u);
}

static void test_center_target_retries_after_temporary_rejection(void)
{
    const stop_test_status_t *status;

    reset_mocks();
    assert(0u != stop_test_app_start());
    mock_now_ms = 1000u;
    mock_balance.estimated_position_m = STOP_TEST_POSITIVE_TARGET_M;
    mock_balance.estimated_velocity_mps = 0.0f;
    stop_test_app_process();
    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    stop_test_app_process();
    mock_now_ms = 2000u;
    mock_balance.estimated_position_m = STOP_TEST_NEGATIVE_TARGET_M;
    stop_test_app_process();
    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    stop_test_app_process();
    status = stop_test_app_get_status();
    assert(status->state == STOP_TEST_COMPLETE);

    mock_target_result = 0u;
    stop_test_app_stop();
    assert(status->state == STOP_TEST_RETURN_CENTER);
    assert(mock_target_count == 2u);
    mock_target_result = 1u;
    mock_now_ms += STOP_TEST_CENTER_RETRY_MS;
    stop_test_app_process();
    assert(mock_target_count == 3u);
    assert(mock_targets[2] == 0.0f);
    assert(status->target_position_m == 0.0f);
}

static void test_waits_for_balance_before_starting_timer(void)
{
    reset_mocks();
    mock_balance.state = BALANCE_SIMPLE_WAIT_VISION;
    mock_balance.flags = 0u;
    assert(0u != stop_test_app_start());
    assert(stop_test_app_get_status()->state == STOP_TEST_WAIT_BALANCE);
    assert(mock_target_count == 0u);

    mock_now_ms = 3000u;
    mock_balance.state = BALANCE_SIMPLE_ACTIVE;
    mock_balance.flags = BALANCE_SIMPLE_FLAG_OBSERVER_VALID |
                         BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    stop_test_app_process();
    assert(stop_test_app_get_status()->state == STOP_TEST_TO_POSITIVE);
    assert(stop_test_app_get_status()->elapsed_ms == 0u);
    assert(mock_target_count == 1u);
}

static void test_plus_four_centimeters_is_not_endpoint_arrival(void)
{
    reset_mocks();
    assert(0u != stop_test_app_start());
    mock_now_ms = 1000u;
    mock_balance.estimated_position_m = 0.040f;
    mock_balance.estimated_velocity_mps = 0.0f;
    stop_test_app_process();
    assert(stop_test_app_get_status()->state == STOP_TEST_TO_POSITIVE);
    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    stop_test_app_process();
    assert(stop_test_app_get_status()->state == STOP_TEST_TO_POSITIVE);
    assert(mock_target_count == 1u);
}

static void test_motion_timeout_returns_target_to_center(void)
{
    const stop_test_status_t *status;

    reset_mocks();
    assert(0u != stop_test_app_start());
    mock_now_ms = STOP_TEST_TIMEOUT_MS + 1u;
    stop_test_app_process();
    status = stop_test_app_get_status();
    assert(status->state == STOP_TEST_TIMEOUT);
    assert(status->stop_reason == STOP_TEST_STOP_TIMEOUT);
    assert(status->time_requirement_met == 0u);
    assert(status->overall_requirement_met == 0u);
    assert(mock_target_count == 2u);
    assert(mock_targets[1] == 0.0f);
}

static void test_completion_at_exactly_five_seconds_is_allowed(void)
{
    reset_mocks();
    assert(0u != stop_test_app_start());

    mock_now_ms = 1000u;
    mock_balance.estimated_position_m = STOP_TEST_POSITIVE_TARGET_M;
    mock_balance.estimated_velocity_mps = 0.0f;
    stop_test_app_process();
    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    stop_test_app_process();

    mock_now_ms = STOP_TEST_TIMEOUT_MS - STOP_TEST_ENDPOINT_SETTLE_MS;
    mock_balance.estimated_position_m = STOP_TEST_NEGATIVE_TARGET_M;
    stop_test_app_process();
    mock_now_ms = STOP_TEST_TIMEOUT_MS;
    stop_test_app_process();

    assert(stop_test_app_get_status()->state == STOP_TEST_COMPLETE);
    assert(stop_test_app_get_status()->elapsed_ms == STOP_TEST_TIMEOUT_MS);
    assert(stop_test_app_get_status()->time_requirement_met != 0u);
    assert(stop_test_app_get_status()->overall_requirement_met != 0u);
}

static void test_user_stop_and_balance_fault(void)
{
    reset_mocks();
    assert(0u != stop_test_app_start());
    mock_now_ms = 100u;
    stop_test_app_stop();
    assert(stop_test_app_get_status()->state == STOP_TEST_USER_STOP);
    assert(mock_targets[mock_target_count - 1u] == 0.0f);
    assert(mock_stop_test_mode == 0u);
    assert(mock_stop_test_mode_disable_count == 1u);

    reset_mocks();
    assert(0u != stop_test_app_start());
    mock_balance.state = BALANCE_SIMPLE_FAULT;
    stop_test_app_process();
    assert(stop_test_app_get_status()->state == STOP_TEST_FAULT);
    assert(stop_test_app_get_status()->stop_reason ==
           STOP_TEST_STOP_BALANCE);
    assert(mock_stop_test_mode == 0u);
    assert(mock_stop_test_mode_disable_count == 1u);
}

static void test_rejects_balance_start_failure(void)
{
    reset_mocks();
    mock_balance_start_result = 0u;
    assert(0u == stop_test_app_start());
    assert(stop_test_app_get_status()->state == STOP_TEST_IDLE);
    assert(stop_test_app_get_status()->stop_reason ==
           STOP_TEST_STOP_START_REJECTED);
    assert(mock_stop_test_mode == 0u);
    assert(mock_stop_test_mode_enable_count == 0u);
    assert(mock_stop_test_mode_disable_count == 0u);
}

static void test_runtime_endpoint_targets_are_used(void)
{
    reset_mocks();
    stop_test_app_set_positive_target_m(0.040f);
    stop_test_app_set_negative_target_m(-0.060f);
    assert(stop_test_app_get_positive_target_m() == 0.040f);
    assert(stop_test_app_get_negative_target_m() == -0.060f);

    assert(0u != stop_test_app_start());
    assert(fabsf(mock_targets[0] - 0.040f) < 0.0001f);
    mock_now_ms = 1000u;
    mock_balance.estimated_position_m = 0.040f;
    mock_balance.estimated_velocity_mps = 0.0f;
    stop_test_app_process();
    mock_now_ms += STOP_TEST_ENDPOINT_SETTLE_MS;
    stop_test_app_process();
    assert(mock_target_count == 2u);
    assert(fabsf(mock_targets[1] - (-0.060f)) < 0.0001f);

    stop_test_app_set_positive_target_m(STOP_TEST_POSITIVE_TARGET_M);
    stop_test_app_set_negative_target_m(STOP_TEST_NEGATIVE_TARGET_M);
}

int main(void)
{
    test_start_enables_stop_test_tuning_after_balance_init_order();
    test_complete_plus_five_to_minus_five();
    test_center_target_retries_after_temporary_rejection();
    test_waits_for_balance_before_starting_timer();
    test_plus_four_centimeters_is_not_endpoint_arrival();
    test_motion_timeout_returns_target_to_center();
    test_completion_at_exactly_five_seconds_is_allowed();
    test_user_stop_and_balance_fault();
    test_rejects_balance_start_failure();
    test_runtime_endpoint_targets_are_used();
    puts("stop test app tests passed");
    return 0;
}
