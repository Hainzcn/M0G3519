#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "balance_app.h"
#include "balance_linkage.h"
#include "control_config.h"
#include "emm42.h"
#include "imu.h"
#include "motor_app.h"
#include "vision_link.h"
#include "wheel_speed_control.h"

/*
 * Black-box regression fixture for failures observed on the rig. This file is
 * deliberately separate from test_balance_app.c so the scenarios can model
 * realistic vision and ACK timing without changing the production modules.
 */

static uint32 mock_now_ms;
static emm42_frame_t mock_frame;
static uint8 mock_frame_ready;
static float mock_queued_position_deg;
static uint32 mock_move_count;
static uint32 mock_position_query_count;
static float mock_last_move_deg;
static uint8 mock_last_command;
static uint8 mock_vision_online;
static uint8 mock_vision_has_snapshot;
static vision_link_snapshot_t mock_vision_snapshot;
static wheel_speed_control_status_t mock_wheel_status;
static imu_snapshot_t mock_imu_snapshot;
static motor_app_mode_enum mock_motor_mode;
static uint32 failures;

#define CHECK(condition, message)                                             \
    do                                                                        \
    {                                                                         \
        if (!(condition))                                                     \
        {                                                                     \
            fprintf(stderr, "FAIL: %s\n", (message));                        \
            failures++;                                                       \
        }                                                                     \
    } while (0)

const wheel_speed_control_status_t *wheel_speed_control_get_status(void)
{
    return &mock_wheel_status;
}

motor_app_mode_enum motor_app_get_mode(void)
{
    return mock_motor_mode;
}

void imu_get_snapshot(imu_snapshot_t *snapshot)
{
    *snapshot = mock_imu_snapshot;
}

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

void emm42_init(void)
{
}

uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    (void)address;
    (void)enabled;
    (void)synchronized;
    mock_last_command = 0xF3u;
    return 1u;
}

uint8 emm42_set_current_position_zero(uint8 address)
{
    (void)address;
    mock_last_command = 0x0Au;
    return 1u;
}

uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized)
{
    (void)address;
    (void)rpm;
    (void)acceleration;
    (void)mode;
    (void)synchronized;
    mock_last_move_deg = angle_deg;
    mock_last_command = 0xFDu;
    mock_move_count++;
    return 1u;
}

uint8 emm42_query_position(uint8 address)
{
    (void)address;
    mock_last_command = 0x36u;
    mock_position_query_count++;
    return 1u;
}

uint8 emm42_stop(uint8 address, uint8 synchronized)
{
    (void)address;
    (void)synchronized;
    return 1u;
}

uint8 emm42_read_frame(emm42_frame_t *frame)
{
    if (0u == mock_frame_ready) return 0u;
    *frame = mock_frame;
    mock_frame_ready = 0u;
    return 1u;
}

uint8 emm42_decode_ack(const emm42_frame_t *frame, uint8 address,
                       uint8 command, uint8 *status)
{
    if ((4u != frame->length) || (address != frame->data[0]) ||
        (command != frame->data[1]))
    {
        return 0u;
    }
    *status = frame->data[2];
    return 1u;
}

uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    if ((8u != frame->length) || (address != frame->data[0]) ||
        (0x36u != frame->data[1]))
    {
        return 0u;
    }
    *position_deg = mock_queued_position_deg;
    return 1u;
}

uint32 emm42_get_rx_overflow_count(void)
{
    return 0u;
}

void vision_link_get_status(vision_link_status_t *status)
{
    status->link_online = mock_vision_online;
}

uint8 vision_link_get_latest_snapshot(vision_link_snapshot_t *snapshot)
{
    if (0u == mock_vision_has_snapshot) return 0u;
    *snapshot = mock_vision_snapshot;
    return 1u;
}

uint8 vision_link_take_new_valid_measurement(vision_link_snapshot_t *snapshot)
{
    (void)snapshot;
    return 0u;
}

static float level_motor_position(void)
{
    float position_deg = 0.0f;
    CHECK(0u != balance_linkage_motor_from_physical_lever_deg(
                    0.0f, &position_deg),
          "level linkage conversion must be valid");
    return position_deg;
}

static void queue_ack_status(uint8 command, uint8 status)
{
    mock_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    mock_frame.data[1] = command;
    mock_frame.data[2] = status;
    mock_frame.data[3] = 0x6Bu;
    mock_frame.length = 4u;
    mock_frame_ready = 1u;
}

static void queue_ack(uint8 command)
{
    queue_ack_status(command, 0x02u);
}

static void queue_position(float position_deg)
{
    mock_queued_position_deg = position_deg;
    mock_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    mock_frame.data[1] = 0x36u;
    mock_frame.data[7] = 0x6Bu;
    mock_frame.length = 8u;
    mock_frame_ready = 1u;
}

static void reset_fixture(void)
{
    memset(&mock_frame, 0, sizeof(mock_frame));
    memset(&mock_vision_snapshot, 0, sizeof(mock_vision_snapshot));
    memset(&mock_wheel_status, 0, sizeof(mock_wheel_status));
    memset(&mock_imu_snapshot, 0, sizeof(mock_imu_snapshot));
    mock_now_ms = 0u;
    mock_frame_ready = 0u;
    mock_queued_position_deg = 0.0f;
    mock_move_count = 0u;
    mock_position_query_count = 0u;
    mock_last_move_deg = 0.0f;
    mock_last_command = 0u;
    mock_vision_online = 0u;
    mock_vision_has_snapshot = 0u;
    mock_motor_mode = MOTOR_APP_MODE_DISABLED;
}

static void publish_vision(int16 position_dmm, int16 velocity_mm_s)
{
    mock_vision_online = 1u;
    mock_vision_has_snapshot = 1u;
    mock_vision_snapshot.flags = VISION_LINK_FLAG_MEASURED_VALID |
                                 VISION_LINK_FLAG_TRACKER_READY |
                                 VISION_LINK_FLAG_CALIBRATION_VALID;
    mock_vision_snapshot.sequence++;
    mock_vision_snapshot.position_dmm = position_dmm;
    mock_vision_snapshot.velocity_mm_s = velocity_mm_s;
    mock_vision_snapshot.confidence = 80u;
    mock_vision_snapshot.boot_id = 1u;
    mock_vision_snapshot.processing_ms = 0u;
    mock_vision_snapshot.received_ms = mock_now_ms;
}

static void answer_pending_command(void)
{
    uint8 attempts = 0u;
    while ((0u != (balance_app_get_status()->flags &
                   BALANCE_APP_FLAG_COMMAND_PENDING)) &&
           (attempts < 4u))
    {
        if (0x36u == mock_last_command)
            queue_position(mock_last_move_deg);
        else
            queue_ack(mock_last_command);
        balance_app_process();
        attempts++;
    }
}

static uint8 start_to_wait_vision(void)
{
    reset_fixture();
    balance_app_init();
    mock_now_ms = BALANCE_POWER_WAIT_MS;
    balance_app_process();
    queue_ack(0xF3u);
    balance_app_process();
    mock_now_ms += BALANCE_LOWER_STOP_SETTLE_MS;
    balance_app_process();
    queue_ack(0x0Au);
    balance_app_process();
    queue_ack(0xF3u);
    balance_app_process();
    queue_ack(0xFDu);
    balance_app_process();
    queue_position(level_motor_position());
    balance_app_process();
    mock_now_ms += BALANCE_LEVEL_SETTLE_MS;
    balance_app_process();
    queue_position(level_motor_position());
    balance_app_process();
    CHECK(BALANCE_APP_WAIT_VISION == balance_app_get_status()->state,
          "startup must reach WAIT_VISION");
    return (BALANCE_APP_WAIT_VISION == balance_app_get_status()->state) ?
        1u : 0u;
}

static uint8 activate_at_40hz(int16 position_dmm, int16 velocity_mm_s)
{
    uint8 index;
    for (index = 0u; index < BALANCE_RECOVERY_VALID_FRAMES; index++)
    {
        mock_now_ms += 25u;
        publish_vision(position_dmm, velocity_mm_s);
        balance_app_process();
        answer_pending_command();
        if (index + 1u < BALANCE_RECOVERY_VALID_FRAMES)
        {
            CHECK(BALANCE_APP_ACTIVE != balance_app_get_status()->state,
                  "fewer than five distinct vision frames must not activate");
        }
    }
    CHECK(BALANCE_APP_ACTIVE == balance_app_get_status()->state,
          "five distinct 40 Hz frames must activate control");
    return (BALANCE_APP_ACTIVE == balance_app_get_status()->state) ? 1u : 0u;
}

static void run_40hz(uint32 duration_ms, int16 position_dmm,
                     int16 velocity_mm_s, uint8 answer_commands)
{
    uint32 elapsed;
    for (elapsed = 5u; elapsed <= duration_ms; elapsed += 5u)
    {
        mock_now_ms += 5u;
        if (0u == (elapsed % 25u))
            publish_vision(position_dmm, velocity_mm_s);
        balance_app_process();
        if (0u != answer_commands) answer_pending_command();
    }
}

static void test_40hz_vision_cadence(void)
{
    uint16 starting_sequence;
    uint32 starting_failures = failures;

    if ((0u == start_to_wait_vision()) || (0u == activate_at_40hz(0, 0)))
        return;
    starting_sequence = balance_app_get_status()->vision_sequence;
    run_40hz(1000u, 0, 0, 1u);
    CHECK(BALANCE_APP_ACTIVE == balance_app_get_status()->state,
          "40 Hz vision must keep the app ACTIVE");
    CHECK((uint16)(balance_app_get_status()->vision_sequence -
                   starting_sequence) == 40u,
          "one second at 40 Hz must accept exactly 40 new sequences");
    CHECK(balance_app_get_status()->vision_age_ms <= 28u,
          "40 Hz vision age, including 3 ms transport compensation, must stay fresh");
    CHECK(0u != (balance_app_get_status()->control_flags &
                 BALANCE_CONTROL_FLAG_MEASUREMENT_FRESH),
          "40 Hz zero-latency measurements must remain fresh");
    if (failures == starting_failures) puts("PASS: 40 Hz vision cadence");
}

static balance_control_config_t predictor_regression_config(void)
{
    balance_control_config_t config;
    memset(&config, 0, sizeof(config));
    config.position_gain_s_inv = 1.0f;
    config.velocity_gain_s_inv = 5.0f;
    config.max_ball_velocity_mps = 0.030f;
    config.rolling_factor = BALANCE_ROLLING_FACTOR;
    config.rolling_friction_accel_mps2 =
        BALANCE_ROLLING_FRICTION_ACCEL_MPS2;
    config.rail_curvature_m_inv = BALANCE_RAIL_CURVATURE_M_INV;
    config.position_correction_gain = BALANCE_ESTIMATOR_POSITION_GAIN;
    config.velocity_residual_gain =
        BALANCE_ESTIMATOR_VELOCITY_RESIDUAL_GAIN;
    config.max_ball_accel_mps2 = BALANCE_MAX_BALL_ACCEL_MPS2;
    config.brake_accel_mps2 = BALANCE_BRAKE_ACCEL_MPS2;
    config.actuator_delay_s = 0.120f;
    config.brake_margin_delay_s = 0.020f;
    config.overspeed_release_ratio = BALANCE_OVERSPEED_RELEASE_RATIO;
    config.overspeed_min_hold_ms = BALANCE_OVERSPEED_MIN_HOLD_MS;
    config.command_period_s = 0.020f;
    config.capture_position_m = 0.004f;
    config.center_dead_position_m = 0.0015f;
    config.capture_velocity_mps = 0.004f;
    config.stick_velocity_mps = 0.003f;
    config.capture_integral_gain = BALANCE_CAPTURE_INTEGRAL_GAIN;
    config.capture_max_accel_mps2 = BALANCE_CAPTURE_MAX_ACCEL_MPS2;
    config.breakaway_angle_deg = BALANCE_BREAKAWAY_ANGLE_DEG;
    config.breakaway_qualify_ms = BALANCE_BREAKAWAY_QUALIFY_MS;
    config.breakaway_pulse_ms = BALANCE_BREAKAWAY_PULSE_MS;
    config.breakaway_movement_m = BALANCE_BREAKAWAY_MOVEMENT_M;
    config.max_lever_angle_deg = BALANCE_MAX_LEVER_ANGLE_DEG;
    config.degraded_lever_angle_deg = BALANCE_DEGRADED_LEVER_ANGLE_DEG;
    config.edge_recovery_accel_mps2 = BALANCE_EDGE_RECOVERY_ACCEL_MPS2;
    config.edge_position_m = BALANCE_EDGE_POSITION_M;
    config.hard_edge_position_m = BALANCE_HARD_EDGE_POSITION_M;
    config.fresh_measurement_ms = BALANCE_FRESH_MEASUREMENT_MS;
    config.valid_measurement_ms = BALANCE_VALID_MEASUREMENT_MS;
    return config;
}

static void test_wrong_120ms_prediction_cannot_capture_fast_state(void)
{
    balance_control_t control;
    balance_control_config_t config = predictor_regression_config();
    balance_control_input_t input;
    const balance_control_output_t *output;
    uint8 index;
    uint32 starting_failures = failures;

    balance_control_init(&control, &config);
    memset(&input, 0, sizeof(input));
    input.dt_s = 0.005f;
    input.actuator_command_updated = 1u;
    input.actuator_command_angle_deg = -1.4f;
    for (index = 0u; index < 7u; index++)
        balance_control_step(&control, &input);

    memset(&input, 0, sizeof(input));
    input.dt_s = 0.005f;
    input.new_measurement = 1u;
    input.measurement_valid = 1u;
    input.measured_position_m = 0.004f;
    input.measured_velocity_mps = -0.030f;
    input.measurement_interval_s = 0.025f;
    input.measurement_age_ms = 3u;
    input.target_position_m = 0.0f;
    input.reference_position_m = 0.0f;
    input.reference_holding = 1u;
    input.actual_lever_valid = 1u;
    input.actual_lever_angle_deg = 0.0f;
    input.update_control_output = 1u;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);

    CHECK(fabsf(output->estimated_velocity_mps) >= 0.020f,
          "test setup requires a fast current estimate");
    CHECK(fabsf(output->predicted_velocity_mps) <=
              config.capture_velocity_mps,
          "test setup requires the wrong 120 ms prediction inside capture velocity");
    CHECK(fabsf(output->position_error_m) <= config.capture_position_m,
          "test setup requires predicted position inside capture band");
    CHECK(BALANCE_CONTROL_PHASE_CAPTURE != output->phase &&
              BALANCE_CONTROL_PHASE_HOLD != output->phase,
          "a low delayed prediction must not capture/hold while the current estimate is fast");
    if (failures == starting_failures)
        puts("PASS: wrong 120 ms prediction is capture-safe");
}

static void test_recovery_neutral_and_40hz_reacquisition(void)
{
    uint8 index;
    uint32 recovery_start_ms;
    uint32 starting_failures = failures;
    const balance_app_status_t *status;

    if ((0u == start_to_wait_vision()) ||
        (0u == activate_at_40hz(800, 0)))
    {
        return;
    }
    run_40hz(300u, 800, 0, 1u);
    mock_vision_online = 0u;
    mock_now_ms += 5u;
    balance_app_process();
    answer_pending_command();
    CHECK(BALANCE_APP_RECOVERY == balance_app_get_status()->state,
          "vision link loss must enter RECOVERY");
    recovery_start_ms = mock_now_ms;

    for (index = 0u; index < 60u; index++)
    {
        mock_now_ms += 5u;
        balance_app_process();
        answer_pending_command();
    }
    status = balance_app_get_status();
    CHECK(BALANCE_APP_RECOVERY == status->state,
          "stale snapshot must not leave RECOVERY");
    CHECK(fabsf(status->lever_angle_deg) <=
              BALANCE_LEVER_COMMAND_DEADBAND_DEG,
          "RECOVERY must command a neutral lever instead of retaining position tilt");

    mock_vision_online = 1u;
    for (index = 0u; index < BALANCE_RECOVERY_VALID_FRAMES; index++)
    {
        mock_now_ms += 25u;
        publish_vision(-800, 30);
        balance_app_process();
        answer_pending_command();
        if (index + 1u < BALANCE_RECOVERY_VALID_FRAMES)
        {
            CHECK(BALANCE_APP_RECOVERY == balance_app_get_status()->state,
                  "RECOVERY must require five distinct reacquisition frames");
        }
    }
    status = balance_app_get_status();
    CHECK(BALANCE_APP_ACTIVE == status->state,
          "fifth distinct 40 Hz frame must reactivate control");
    CHECK((mock_now_ms - recovery_start_ms) >= 400u,
          "reactivation timing must include recovery dwell plus five 40 Hz frames");
    CHECK(fabsf(status->estimated_position_m + 0.080f) <= 0.005f,
          "reactivation estimator must be anchored to the newly acquired ball");
    CHECK(BALANCE_CONTROL_PHASE_CAPTURE != status->control_phase &&
              BALANCE_CONTROL_PHASE_HOLD != status->control_phase,
          "30 mm/s reacquisition must not immediately capture/hold");
    if (failures == starting_failures)
        puts("PASS: RECOVERY neutral and 40 Hz reacquisition");
}

static void test_deadband_and_delayed_ack_coalescing(void)
{
    uint32 move_count;
    uint32 wait_ms;
    uint32 starting_failures = failures;

    if ((0u == start_to_wait_vision()) || (0u == activate_at_40hz(0, 0)))
        return;
    answer_pending_command();
    move_count = mock_move_count;
    run_40hz(200u, 1, 0, 1u);
    CHECK(mock_move_count == move_count,
          "sub-deadband center correction must not emit MOVE commands");

    for (wait_ms = 0u; wait_ms < 250u && mock_move_count == move_count;
         wait_ms += 5u)
    {
        mock_now_ms += 5u;
        if (0u == (wait_ms % 25u)) publish_vision(300, 0);
        balance_app_process();
        if ((0u != (balance_app_get_status()->flags &
                    BALANCE_APP_FLAG_COMMAND_PENDING)) &&
            (0x36u == mock_last_command))
        {
            queue_position(mock_last_move_deg);
            balance_app_process();
        }
    }
    CHECK(mock_move_count == move_count + 1u,
          "super-deadband correction must emit one MOVE command");
    if (mock_move_count != move_count + 1u) return;
    CHECK(0xFDu == mock_last_command,
          "the pending super-deadband command must be MOVE");

    move_count = mock_move_count;
    for (wait_ms = 0u; wait_ms < 30u; wait_ms += 5u)
    {
        mock_now_ms += 5u;
        if (0u == (wait_ms % 25u)) publish_vision(350, 0);
        balance_app_process();
    }
    CHECK(mock_move_count == move_count,
          "no duplicate MOVE may be sent while its ACK is pending");
    CHECK(BALANCE_APP_FAULT != balance_app_get_status()->state,
          "a valid ACK delay below timeout must not fault");

    queue_ack(0xF3u);
    balance_app_process();
    CHECK(0u != (balance_app_get_status()->flags &
                 BALANCE_APP_FLAG_COMMAND_PENDING),
          "an ACK for the wrong command must not clear pending MOVE");
    mock_now_ms += 5u;
    queue_ack(0xFDu);
    balance_app_process();
    CHECK(BALANCE_APP_FAULT != balance_app_get_status()->state,
          "correct delayed MOVE ACK must be accepted without fault");
    CHECK(0u == balance_app_get_status()->command_error_count,
          "wrong-command noise followed by correct ACK must not count as rejection");
    answer_pending_command();
    if (failures == starting_failures)
        puts("PASS: deadband and delayed ACK coalescing");
}

int main(void)
{
#if (BALANCE_STARTUP_CALIBRATED == 0u)
    puts("balance failure regressions skipped: startup is uncalibrated");
    return 0;
#else
    test_40hz_vision_cadence();
    test_wrong_120ms_prediction_cannot_capture_fast_state();
    test_recovery_neutral_and_40hz_reacquisition();
    test_deadband_and_delayed_ack_coalescing();
    if (0u != failures)
    {
        fprintf(stderr, "balance failure regressions: %lu failure(s)\n",
                (unsigned long)failures);
        return 1;
    }
    puts("balance failure regressions passed");
    return 0;
#endif
}
