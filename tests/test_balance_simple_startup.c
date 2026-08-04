#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "balance_simple_app.h"
#include "control_config.h"
#include "emm42.h"
#include "vision_link.h"

static uint32 mock_now_ms;
static uint32 mock_enable_count;
static uint32 mock_zero_count;
static uint32 mock_move_count;
static uint32 mock_position_query_count;
static uint32 mock_fallback_log_count;
static uint32 mock_open_loop_level_log_count;
static uint32 mock_stop_count;
static uint32 mock_velocity_command_count;
static float mock_move_angle_deg;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    if (NULL == message)
    {
        return;
    }
    if (NULL != strstr(message, "startup ACK timeout"))
    {
        mock_fallback_log_count++;
    }
    if (NULL != strstr(message, "level assumed"))
    {
        mock_open_loop_level_log_count++;
    }
}

uint8 vision_link_take_new_valid_measurement(vision_link_snapshot_t *snapshot)
{
    (void)snapshot;
    return 0u;
}

float vision_link_correct_position_m(int16 position_dmm)
{
    return (float)position_dmm * 0.0001f +
        BALANCE_VISION_POSITION_OFFSET_M;
}

void emm42_init(void)
{
}

uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    assert(EMM42_DEFAULT_ADDRESS == address);
    (void)enabled;
    (void)synchronized;
    mock_enable_count++;
    return 1u;
}

uint8 emm42_set_current_position_zero(uint8 address)
{
    assert(EMM42_DEFAULT_ADDRESS == address);
    mock_zero_count++;
    return 1u;
}

uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized)
{
    assert(EMM42_DEFAULT_ADDRESS == address);
    assert(BALANCE_LEVEL_RETURN_RPM == rpm);
    assert(BALANCE_SIMPLE_EMM42_ACCELERATION == acceleration);
    assert(EMM42_POSITION_ABSOLUTE == mode);
    assert(0u == synchronized);
    mock_move_count++;
    mock_move_angle_deg = angle_deg;
    return 1u;
}

uint8 emm42_query_position(uint8 address)
{
    assert(EMM42_DEFAULT_ADDRESS == address);
    mock_position_query_count++;
    return 1u;
}

uint8 emm42_query_velocity(uint8 address)
{
    (void)address;
    return 1u;
}

uint8 emm42_run_velocity(uint8 address, int16 rpm, uint8 acceleration,
                         uint8 synchronized)
{
    (void)address;
    (void)rpm;
    (void)acceleration;
    (void)synchronized;
    mock_velocity_command_count++;
    return 1u;
}

uint8 emm42_stop(uint8 address, uint8 synchronized)
{
    (void)address;
    (void)synchronized;
    mock_stop_count++;
    return 1u;
}

uint8 emm42_read_frame(emm42_frame_t *frame)
{
    (void)frame;
    return 0u;
}

uint8 emm42_decode_ack(const emm42_frame_t *frame, uint8 address,
                       uint8 command, uint8 *status)
{
    (void)frame;
    (void)address;
    (void)command;
    (void)status;
    return 0u;
}

uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    (void)frame;
    (void)address;
    (void)position_deg;
    return 0u;
}

uint8 emm42_decode_velocity_rpm(const emm42_frame_t *frame, uint8 address,
                                int16 *velocity_rpm)
{
    (void)frame;
    (void)address;
    (void)velocity_rpm;
    return 0u;
}

uint32 emm42_get_rx_overflow_count(void)
{
    return 0u;
}

static void process_at(uint32 now_ms)
{
    mock_now_ms = now_ms;
    balance_simple_app_process();
}

static void reset_mocks(void)
{
    mock_now_ms = 0u;
    mock_enable_count = 0u;
    mock_zero_count = 0u;
    mock_move_count = 0u;
    mock_position_query_count = 0u;
    mock_fallback_log_count = 0u;
    mock_open_loop_level_log_count = 0u;
    mock_stop_count = 0u;
    mock_velocity_command_count = 0u;
    mock_move_angle_deg = 0.0f;
}

static void run_startup_without_uart_responses(void)
{
    process_at(BALANCE_POWER_WAIT_MS);
    assert(1u == mock_enable_count);

    process_at(BALANCE_POWER_WAIT_MS + BALANCE_SIMPLE_COMMAND_TIMEOUT_MS);
    process_at(BALANCE_POWER_WAIT_MS + BALANCE_SIMPLE_COMMAND_TIMEOUT_MS +
               BALANCE_LOWER_STOP_SETTLE_MS);
    assert(1u == mock_zero_count);

    process_at(BALANCE_POWER_WAIT_MS + 2u * BALANCE_SIMPLE_COMMAND_TIMEOUT_MS +
               BALANCE_LOWER_STOP_SETTLE_MS);
    assert(2u == mock_enable_count);

    process_at(BALANCE_POWER_WAIT_MS + 3u * BALANCE_SIMPLE_COMMAND_TIMEOUT_MS +
               BALANCE_LOWER_STOP_SETTLE_MS);
    assert(1u == mock_move_count);
    assert((mock_move_angle_deg > -18.61f) &&
           (mock_move_angle_deg < -18.59f));

    process_at(BALANCE_POWER_WAIT_MS + 4u * BALANCE_SIMPLE_COMMAND_TIMEOUT_MS +
               BALANCE_LOWER_STOP_SETTLE_MS);
    assert(0u != mock_position_query_count);

    process_at(BALANCE_POWER_WAIT_MS + 4u * BALANCE_SIMPLE_COMMAND_TIMEOUT_MS +
               BALANCE_LOWER_STOP_SETTLE_MS +
               BALANCE_SIMPLE_STARTUP_OPEN_LOOP_LEVEL_MS);
}

static void test_startup_continues_without_uart_responses(void)
{
    const balance_simple_status_t *status;

    reset_mocks();
    balance_simple_app_init();
    assert(BALANCE_SIMPLE_STARTUP_LEVEL ==
           balance_simple_app_get_status()->state);
    run_startup_without_uart_responses();

    status = balance_simple_app_get_status();
    assert(BALANCE_SIMPLE_WAIT_VISION == status->state);
    assert(BALANCE_SIMPLE_FAULT_NONE == status->fault);
    assert(4u == mock_fallback_log_count);
    assert(1u == mock_open_loop_level_log_count);
}

static void test_disable_waits_for_startup_level(void)
{
    reset_mocks();
    balance_simple_app_init();
    balance_simple_app_disable();
    assert(BALANCE_SIMPLE_STARTUP_LEVEL ==
           balance_simple_app_get_status()->state);

    run_startup_without_uart_responses();

    assert(BALANCE_SIMPLE_DISABLED ==
           balance_simple_app_get_status()->state);
    assert(BALANCE_SIMPLE_FAULT_NONE ==
           balance_simple_app_get_status()->fault);
    assert(1u == mock_move_count);
}

static void test_capture_level_stops_before_absolute_move_and_holds_position(void)
{
    const balance_simple_status_t *status;
    uint32 capture_start_ms;

    reset_mocks();
    balance_simple_app_init();
    run_startup_without_uart_responses();
    status = balance_simple_app_get_status();
    assert(BALANCE_SIMPLE_WAIT_VISION == status->state);
    assert(0u != balance_simple_app_set_target_position_m(0.040f));

    mock_stop_count = 0u;
    mock_velocity_command_count = 0u;
    capture_start_ms = mock_now_ms;
    assert(0u != balance_simple_app_prepare_capture());
    assert(BALANCE_SIMPLE_STARTUP_LEVEL == status->state);
    assert(mock_stop_count == 0u);

    process_at(capture_start_ms + 1u);
    assert(mock_stop_count == 1u);
    assert(mock_move_count == 1u);

    process_at(capture_start_ms + 1u +
               BALANCE_SIMPLE_COMMAND_TIMEOUT_MS);
    assert(mock_move_count == 2u);
    assert((mock_move_angle_deg > -18.61f) &&
           (mock_move_angle_deg < -18.59f));

    process_at(capture_start_ms + 1u +
               2u * BALANCE_SIMPLE_COMMAND_TIMEOUT_MS);
    process_at(capture_start_ms + 1u +
               2u * BALANCE_SIMPLE_COMMAND_TIMEOUT_MS +
               BALANCE_SIMPLE_STARTUP_OPEN_LOOP_LEVEL_MS);
    assert(BALANCE_SIMPLE_WAIT_VISION == status->state);
    assert(mock_velocity_command_count == 0u);

    balance_simple_app_cancel_capture();
    assert(status->target_position_m == 0.0f);
    assert(mock_stop_count == 2u);
}

static void test_planned_feedforward_is_immediate_and_imu_is_correction(void)
{
    const balance_simple_status_t *status;
    const float planned_accel_mps2 =
        LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S * 3.14159265f *
        CHASSIS_WHEEL_DIAMETER_M / 60.0f;
    const float preview_accel_mps2 = planned_accel_mps2;
    const float imu_accel_mps2 = 0.0f;
    const float expected_accel_mps2 = preview_accel_mps2 +
        BALANCE_SIMPLE_CAR_FF_IMU_CORRECTION_GAIN *
            (imu_accel_mps2 - planned_accel_mps2);
    const float expected_angle_deg =
        -BALANCE_SIMPLE_CAR_FF_GAIN *
        atan2f(expected_accel_mps2, 9.80665f) *
        (180.0f / 3.14159265f);
    uint32 index;

    reset_mocks();
    balance_simple_app_init();
    status = balance_simple_app_get_status();
    balance_simple_app_set_vehicle_accel_components_mps2(
        planned_accel_mps2, preview_accel_mps2, imu_accel_mps2, 1u);
    assert(status->car_accel_valid != 0u);
    assert(status->car_filtered_accel_mps2 == imu_accel_mps2);
    assert(status->car_feedforward_active != 0u);
    assert(fabsf(status->car_accel_mps2 - expected_accel_mps2) < 0.0001f);
    assert(fabsf(status->car_feedforward_angle_deg - expected_angle_deg) <
           0.0001f);
    assert(status->car_feedforward_angle_deg < 0.0f);
    assert(BALANCE_SIMPLE_CAR_FF_GAIN == 1.10f);
    assert(BALANCE_SIMPLE_BEAM_ANGLE_KP_S_INV == 12.0f);
    assert(BALANCE_SIMPLE_CAR_FF_MAX_ANGLE_DEG >= 4.4f);
    assert(fabsf(expected_angle_deg) < BALANCE_SIMPLE_CAR_FF_MAX_ANGLE_DEG);

    for (index = 1u; index <= 40u; index++)
    {
        mock_now_ms += BALANCE_SIMPLE_CONTROL_PERIOD_MS;
        balance_simple_app_set_vehicle_accel_components_mps2(
            0.0f, 0.0f, 0.0f, 1u);
    }
    assert(status->car_feedforward_active == 0u);
    assert(status->car_feedforward_angle_deg == 0.0f);

    balance_simple_app_set_vehicle_accel_mps2(0.0f, 0u);
    assert(status->car_accel_valid == 0u);
    assert(status->car_filtered_accel_mps2 == 0.0f);
}

static void test_wait_vision_accepts_feedforward_only_mode(void)
{
    const balance_simple_status_t *status;

    reset_mocks();
    balance_simple_app_init();
    run_startup_without_uart_responses();
    status = balance_simple_app_get_status();
    assert(status->state == BALANCE_SIMPLE_WAIT_VISION);
    assert(0u != balance_simple_app_set_feedforward_only(1u));
    assert(0u != (status->flags & BALANCE_SIMPLE_FLAG_FEEDFORWARD_ONLY));
    assert(0u != balance_simple_app_set_feedforward_only(0u));
    assert(0u == (status->flags & BALANCE_SIMPLE_FLAG_FEEDFORWARD_ONLY));
}

static void test_fixed_beam_bias_tuning_is_retained_and_limited(void)
{
    balance_simple_app_set_fixed_beam_bias_deg(1.7f);
    balance_simple_app_init();
    assert(fabsf(balance_simple_app_get_fixed_beam_bias_deg() - 1.7f) <
           0.0001f);

    balance_simple_app_set_fixed_beam_bias_deg(100.0f);
    assert(balance_simple_app_get_fixed_beam_bias_deg() ==
           BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG);
    balance_simple_app_set_fixed_beam_bias_deg(-100.0f);
    assert(balance_simple_app_get_fixed_beam_bias_deg() ==
           -BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG);

    balance_simple_app_set_fixed_beam_bias_deg(
        BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG);
}

int main(void)
{
    test_startup_continues_without_uart_responses();
    test_disable_waits_for_startup_level();
    test_capture_level_stops_before_absolute_move_and_holds_position();
    test_planned_feedforward_is_immediate_and_imu_is_correction();
    test_wait_vision_accepts_feedforward_only_mode();
    test_fixed_beam_bias_tuning_is_retained_and_limited();
    puts("balance simple startup fallback tests passed");
    return 0;
}
