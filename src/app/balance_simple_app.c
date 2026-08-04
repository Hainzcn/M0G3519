#include "balance_simple_app.h"

#include <math.h>
#include <string.h>

#include "balance_linkage.h"
#include "balance_velocity_actuator.h"
#include "ball_state_observer.h"
#include "ball_velocity_controller.h"
#include "control_config.h"
#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "vision_link.h"

#define SIMPLE_ADDRESS                 (EMM42_DEFAULT_ADDRESS)
#define SIMPLE_ACK_OK                  (0x02u)
#define SIMPLE_COMMAND_ZERO            (0x0Au)
#define SIMPLE_COMMAND_ENABLE          (0xF3u)
#define SIMPLE_COMMAND_MOVE            (0xFDu)
#define SIMPLE_COMMAND_VELOCITY        (0xF6u)
#define SIMPLE_COMMAND_STOP            (0xFEu)
#define SIMPLE_COMMAND_VELOCITY_QUERY  (0x35u)
#define SIMPLE_COMMAND_POSITION        (0x36u)
#define SIMPLE_INVALID_AGE             (0xFFFFFFFFu)
#define SIMPLE_GRAVITY_MPS2             (9.80665f)
#define SIMPLE_RAD_TO_DEG               (57.29577951308232f)
#define SIMPLE_CONTROLLER_REVISION     "angle-pi-car-ff-v7"
#define SIMPLE_CONTROLLER_LIMIT_FLAGS  ((uint16)( \
    (BALL_VELOCITY_CONTROL_VELOCITY_LIMITED | \
     BALL_VELOCITY_CONTROL_OMEGA_LIMITED | \
     BALL_VELOCITY_CONTROL_ANGLE_LIMITED | \
     BALL_VELOCITY_CONTROL_ANGLE_SLEWED) << 8u))
#define SIMPLE_ACTUATOR_LIMIT_FLAGS    ((uint16)( \
    BALANCE_VELOCITY_ACTUATOR_SOFT_LIMIT | \
    BALANCE_VELOCITY_ACTUATOR_HARD_LIMIT | \
    BALANCE_VELOCITY_ACTUATOR_NO_FEEDBACK))

typedef enum
{
    SIMPLE_START_POWER_WAIT = 0,
    SIMPLE_START_WAIT_DISABLE,
    SIMPLE_START_LOWER_SETTLE,
    SIMPLE_START_WAIT_ZERO,
    SIMPLE_START_WAIT_ENABLE,
    SIMPLE_START_WAIT_LEVEL_COMMAND,
    SIMPLE_START_WAIT_LEVEL,
} simple_startup_stage_enum;

static ball_state_observer_t simple_observer;
static ball_velocity_controller_t simple_controller;
static balance_velocity_actuator_t simple_actuator;
static balance_simple_status_t simple_status;
static emm42_frame_t simple_rx_frame;
static simple_startup_stage_enum simple_startup_stage;
static uint32 simple_state_start_ms;
static uint32 simple_stage_start_ms;
static uint32 simple_last_control_ms;
static uint32 simple_last_position_query_ms;
static uint32 simple_last_velocity_query_ms;
static uint32 simple_last_velocity_send_ms;
static uint32 simple_pending_since_ms;
static uint32 simple_motor_position_ms;
static uint32 simple_motor_velocity_ms;
static uint32 simple_level_stable_start_ms;
#if (BALANCE_SIMPLE_STATIC_LOCK_ENABLE != 0u)
static uint32 simple_static_stable_start_ms;
#endif
static uint32 simple_last_capture_ms;
static float simple_level_motor_deg;
static float simple_measurement_dt_s;
static uint8 simple_pending_command;
static uint8 simple_new_measurement;
static uint8 simple_level_stable;
#if (BALANCE_SIMPLE_STATIC_LOCK_ENABLE != 0u)
static uint8 simple_static_stable;
#endif
static uint8 simple_motor_position_valid;
static uint8 simple_motor_velocity_valid;
static uint8 simple_command_errors;
static uint8 simple_disable_after_startup;
static int16 simple_desired_rpm;
static int16 simple_last_sent_rpm;
static uint8 simple_has_sent_rpm;
static float simple_vehicle_feedforward_angle_deg;
static float simple_vehicle_filtered_accel_mps2;
static uint32 simple_vehicle_filter_ms;
static uint32 simple_vehicle_ff_transition_ms;
static uint8 simple_vehicle_filter_initialized;
static uint8 simple_vehicle_ff_transition_active;
static uint8 simple_vehicle_ff_active;
static uint8 simple_stop_test_mode;

static float simple_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void simple_apply_control_mode(void)
{
    if (0u != simple_stop_test_mode)
    {
        simple_controller.config.position_kp_s_inv =
            STOP_TEST_POSITION_KP_S_INV;
        simple_controller.config.position_ki_s2_inv =
            STOP_TEST_POSITION_KI_S2_INV;
        simple_controller.config.max_target_velocity_mps =
            STOP_TEST_MAX_TARGET_VELOCITY_MPS;
        simple_controller.config.braking_envelope_mps2 =
            STOP_TEST_BRAKING_ENVELOPE_MPS2;
        simple_controller.config.velocity_kv_deg_per_mmps =
            STOP_TEST_VELOCITY_KV_DEG_PER_MM;
        simple_controller.config.near_position_m =
            STOP_TEST_NEAR_POSITION_M;
        simple_controller.config.near_gain = STOP_TEST_NEAR_GAIN;
        simple_controller.config.near_scale_max =
            STOP_TEST_NEAR_SCALE_MAX;
        simple_controller.config.max_target_beam_angle_deg =
            STOP_TEST_MAX_BEAM_ANGLE_DEG;
        simple_controller.config.max_beam_velocity_deg_s =
            STOP_TEST_MAX_BEAM_VELOCITY_DEG_S;
        simple_status.flags |= BALANCE_SIMPLE_FLAG_STOP_TEST_TUNING;
    }
    else
    {
        simple_controller.config.position_kp_s_inv =
            BALANCE_SIMPLE_POSITION_KP_S_INV;
        simple_controller.config.position_ki_s2_inv =
            BALANCE_SIMPLE_POSITION_KI_S2_INV;
        simple_controller.config.max_target_velocity_mps =
            BALANCE_SIMPLE_MAX_TARGET_VELOCITY_MPS;
        simple_controller.config.braking_envelope_mps2 =
            BALANCE_SIMPLE_BRAKING_ENVELOPE_MPS2;
        simple_controller.config.velocity_kv_deg_per_mmps =
            BALANCE_SIMPLE_VELOCITY_KV_DEG_PER_MM;
        simple_controller.config.near_position_m =
            BALANCE_SIMPLE_NEAR_POSITION_M;
        simple_controller.config.near_gain = BALANCE_SIMPLE_NEAR_GAIN;
        simple_controller.config.near_scale_max =
            BALANCE_SIMPLE_NEAR_SCALE_MAX;
        simple_controller.config.max_target_beam_angle_deg =
            BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG;
        simple_controller.config.max_beam_velocity_deg_s =
            BALANCE_SIMPLE_MAX_BEAM_VELOCITY_DEG_S;
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_STOP_TEST_TUNING);
    }
    ball_velocity_controller_reset(&simple_controller);
}

static float simple_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static uint8 simple_integrator_output_is_limited(void)
{
    uint16 limit_flags = SIMPLE_CONTROLLER_LIMIT_FLAGS |
                         SIMPLE_ACTUATOR_LIMIT_FLAGS;

    return (0u != (simple_status.saturation_flags & limit_flags)) ? 1u : 0u;
}

static void simple_zero_control_command(void)
{
    simple_desired_rpm = 0;
    simple_status.target_velocity_mps = 0.0f;
    simple_status.target_beam_angle_deg = simple_status.measured_beam_angle_deg;
    simple_status.beam_angle_error_deg = 0.0f;
    simple_status.integral_velocity_mps = 0.0f;
    simple_status.omega_command_deg_s = 0.0f;
    simple_status.motor_rpm_requested = 0.0f;
    simple_status.motor_rpm_command = 0;
    simple_status.flags &=
        (uint16)(~BALANCE_SIMPLE_FLAG_POSITION_ACTIVE);
}

static void simple_set_state(balance_simple_state_enum state, uint32 now_ms)
{
    simple_status.state = state;
    simple_state_start_ms = now_ms;
#if (BALANCE_SIMPLE_STATIC_LOCK_ENABLE != 0u)
    simple_static_stable = 0u;
#endif
}

static uint8 simple_begin_command(uint8 command, uint8 sent, uint32 now_ms)
{
    if (0u == sent)
    {
        simple_status.command_error_count++;
        return 0u;
    }
    simple_pending_command = command;
    simple_pending_since_ms = now_ms;
    simple_status.flags |= BALANCE_SIMPLE_FLAG_COMMAND_PENDING;
    return 1u;
}

static uint8 simple_command_is_query(uint8 command)
{
    return ((SIMPLE_COMMAND_POSITION == command) ||
            (SIMPLE_COMMAND_VELOCITY_QUERY == command)) ? 1u : 0u;
}

static void simple_cancel_pending_query(void)
{
    if (0u == simple_command_is_query(simple_pending_command))
    {
        return;
    }
    simple_pending_command = 0u;
    simple_rx_frame.length = 0u;
    simple_status.flags &=
        (uint16)(~BALANCE_SIMPLE_FLAG_COMMAND_PENDING);
}

static void simple_accept_command(void)
{
    simple_pending_command = 0u;
    simple_command_errors = 0u;
    simple_status.flags &=
        (uint16)(~BALANCE_SIMPLE_FLAG_COMMAND_PENDING);
}

static void simple_enter_fault(balance_simple_fault_enum fault,
                               uint32 now_ms)
{
    if (BALANCE_SIMPLE_FAULT == simple_status.state)
    {
        return;
    }
    simple_status.fault = fault;
    simple_zero_control_command();
    simple_pending_command = 0u;
    simple_rx_frame.length = 0u;
    simple_status.flags &=
        (uint16)(~BALANCE_SIMPLE_FLAG_COMMAND_PENDING);
    simple_set_state(BALANCE_SIMPLE_FAULT, now_ms);
    if (0u != emm42_stop(SIMPLE_ADDRESS, 0u))
    {
        simple_pending_command = SIMPLE_COMMAND_STOP;
        simple_pending_since_ms = now_ms;
        simple_status.flags |= BALANCE_SIMPLE_FLAG_COMMAND_PENDING;
    }
    heartbeat_hw_uart_send_string("[balance-simple] fault latched\r\n");
}

static void simple_preempt_with_zero(uint32 now_ms)
{
    simple_zero_control_command();
    if ((0u == simple_has_sent_rpm) || (0 == simple_last_sent_rpm))
    {
        return;
    }
    simple_pending_command = 0u;
    simple_rx_frame.length = 0u;
    simple_status.flags &=
        (uint16)(~BALANCE_SIMPLE_FLAG_COMMAND_PENDING);
    if (0u != simple_begin_command(
            SIMPLE_COMMAND_VELOCITY,
            emm42_run_velocity(SIMPLE_ADDRESS, 0,
                               BALANCE_SIMPLE_EMM42_ACCELERATION, 0u),
            now_ms))
    {
        simple_last_sent_rpm = 0;
        simple_last_velocity_send_ms = now_ms;
    }
}

static void simple_set_startup_stage(simple_startup_stage_enum stage,
                                     uint32 now_ms)
{
    simple_startup_stage = stage;
    simple_stage_start_ms = now_ms;
}

static uint8 simple_advance_startup_command(uint8 command, uint32 now_ms)
{
    if (BALANCE_SIMPLE_STARTUP_LEVEL != simple_status.state)
    {
        return 0u;
    }
    if ((SIMPLE_START_WAIT_DISABLE == simple_startup_stage) &&
        (SIMPLE_COMMAND_ENABLE == command))
    {
        simple_set_startup_stage(SIMPLE_START_LOWER_SETTLE, now_ms);
    }
    else if ((SIMPLE_START_WAIT_ZERO == simple_startup_stage) &&
             (SIMPLE_COMMAND_ZERO == command))
    {
        simple_set_startup_stage(SIMPLE_START_WAIT_ENABLE, now_ms);
    }
    else if ((SIMPLE_START_WAIT_ENABLE == simple_startup_stage) &&
             (SIMPLE_COMMAND_ENABLE == command))
    {
        simple_set_startup_stage(SIMPLE_START_WAIT_LEVEL_COMMAND, now_ms);
    }
    else if ((SIMPLE_START_WAIT_LEVEL_COMMAND == simple_startup_stage) &&
             (SIMPLE_COMMAND_MOVE == command))
    {
        simple_set_startup_stage(SIMPLE_START_WAIT_LEVEL, now_ms);
        simple_last_position_query_ms = now_ms -
                                        BALANCE_SIMPLE_POSITION_QUERY_MS;
    }
    else
    {
        return 0u;
    }
    return 1u;
}

static void simple_handle_ack(uint8 command, uint8 ack, uint32 now_ms)
{
    if ((command != simple_pending_command) || (SIMPLE_ACK_OK != ack))
    {
        if (command == simple_pending_command)
        {
            simple_pending_command = 0u;
            simple_status.flags &=
                (uint16)(~BALANCE_SIMPLE_FLAG_COMMAND_PENDING);
            simple_status.command_error_count++;
            if (++simple_command_errors >=
                BALANCE_SIMPLE_MAX_COMMAND_ERRORS)
            {
#if (BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE != 0u)
                simple_enter_fault(BALANCE_SIMPLE_FAULT_COMMAND, now_ms);
#endif
            }
        }
        return;
    }
    simple_accept_command();
    (void)simple_advance_startup_command(command, now_ms);
}

static void simple_drain_motor(uint32 now_ms)
{
    uint8 ack;
    int16 velocity_rpm;
    float position_deg;
    float measured_beam_angle_deg;

    while (0u != emm42_read_frame(&simple_rx_frame))
    {
        if ((0u != simple_pending_command) &&
            (0u != emm42_decode_ack(&simple_rx_frame, SIMPLE_ADDRESS,
                                    simple_pending_command, &ack)))
        {
            simple_handle_ack(simple_pending_command, ack, now_ms);
        }
        else if (0u != emm42_decode_position_deg(
                     &simple_rx_frame, SIMPLE_ADDRESS, &position_deg))
        {
            simple_status.motor_position_deg = position_deg;
            simple_motor_position_ms = now_ms;
            simple_motor_position_valid = 1u;
            simple_status.flags |=
                BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
            if (0u != balance_linkage_physical_lever_from_motor_deg(
                    position_deg, &measured_beam_angle_deg))
            {
                simple_status.measured_beam_angle_deg =
                    measured_beam_angle_deg;
            }
            if (SIMPLE_COMMAND_POSITION == simple_pending_command)
            {
                simple_accept_command();
            }
        }
        else if (0u != emm42_decode_velocity_rpm(
                     &simple_rx_frame, SIMPLE_ADDRESS, &velocity_rpm))
        {
            simple_status.motor_rpm_actual = velocity_rpm;
            simple_motor_velocity_ms = now_ms;
            simple_motor_velocity_valid = 1u;
            simple_status.flags |=
                BALANCE_SIMPLE_FLAG_MOTOR_VELOCITY_VALID;
            if (SIMPLE_COMMAND_VELOCITY_QUERY == simple_pending_command)
            {
                simple_accept_command();
            }
        }
    }
}

static void simple_check_pending_timeout(uint32 now_ms)
{
    uint8 command;

    if ((0u == simple_pending_command) ||
        ((now_ms - simple_pending_since_ms) <
         BALANCE_SIMPLE_COMMAND_TIMEOUT_MS))
    {
        return;
    }
    command = simple_pending_command;
    simple_pending_command = 0u;
    simple_rx_frame.length = 0u;
    simple_status.flags &=
        (uint16)(~BALANCE_SIMPLE_FLAG_COMMAND_PENDING);
    simple_status.command_error_count++;
    if ((SIMPLE_COMMAND_POSITION != command) &&
        (SIMPLE_COMMAND_VELOCITY_QUERY != command))
    {
        simple_command_errors++;
#if (BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE != 0u)
        if (0u != simple_advance_startup_command(command, now_ms))
        {
            simple_command_errors = 0u;
            heartbeat_hw_uart_send_string(
                "[balance-simple] startup ACK timeout; timed fallback\r\n");
            return;
        }
#endif
    }
    if (simple_command_errors >= BALANCE_SIMPLE_MAX_COMMAND_ERRORS)
    {
#if (BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE != 0u)
        simple_enter_fault(BALANCE_SIMPLE_FAULT_COMMAND, now_ms);
#endif
    }
}

static uint8 simple_measurement_acceptable(
    const vision_link_snapshot_t *measurement)
{
    uint8 required = VISION_LINK_FLAG_MEASURED_VALID |
                     VISION_LINK_FLAG_TRACKER_READY |
                     VISION_LINK_FLAG_CALIBRATION_VALID;

    return (((measurement->flags & required) == required) &&
            (measurement->confidence >= BALANCE_SIMPLE_MIN_CONFIDENCE)) ?
           1u : 0u;
}

static void simple_take_vision_measurement(void)
{
    vision_link_snapshot_t measurement;
    uint8 result;
    uint32 capture_delta_ms;

    if (0u == vision_link_take_new_valid_measurement(&measurement))
    {
        return;
    }
    simple_status.vision_sequence = measurement.sequence;
    simple_status.capture_ms = measurement.capture_ms;
    simple_status.vision_confidence = measurement.confidence;
    simple_status.vision_flags = measurement.flags;
    simple_status.raw_position_m =
        (float)measurement.position_dmm * 0.0001f;
    if (0u == simple_measurement_acceptable(&measurement))
    {
        return;
    }

    capture_delta_ms = measurement.capture_ms - simple_last_capture_ms;
    result = ball_state_observer_update(
        &simple_observer,
        simple_status.raw_position_m,
        measurement.capture_ms,
        measurement.received_ms,
        measurement.processing_ms,
        measurement.sequence,
        measurement.boot_id);
    if (0u != (result & BALL_OBSERVER_UPDATE_SESSION_RESET))
    {
        ball_velocity_controller_reset(&simple_controller);
        simple_measurement_dt_s = 0.0f;
    }
    else if (0u != (result & BALL_OBSERVER_UPDATE_ACCEPTED))
    {
        simple_measurement_dt_s = (float)capture_delta_ms * 0.001f;
    }
    if (0u != (result & BALL_OBSERVER_UPDATE_ACCEPTED))
    {
        simple_last_capture_ms = measurement.capture_ms;
        simple_new_measurement = 1u;
    }
}

static uint8 simple_motor_feedback_is_fresh(uint32 now_ms)
{
    if (0u == simple_motor_position_valid)
    {
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID);
        return 0u;
    }
    if ((now_ms - simple_motor_position_ms) >
        BALANCE_SIMPLE_MOTOR_POSITION_MAX_AGE_MS)
    {
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID);
        return 0u;
    }
    simple_status.flags |= BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID;
    return 1u;
}

static void simple_update_motor_velocity_freshness(uint32 now_ms)
{
    if ((0u == simple_motor_velocity_valid) ||
        ((now_ms - simple_motor_velocity_ms) >
         BALANCE_SIMPLE_MOTOR_VELOCITY_MAX_AGE_MS))
    {
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_MOTOR_VELOCITY_VALID);
        return;
    }
    simple_status.flags |= BALANCE_SIMPLE_FLAG_MOTOR_VELOCITY_VALID;
}

static void simple_apply_actuator(float omega_deg_s, uint32 now_ms)
{
    balance_velocity_actuator_input_t actuator_input;
    const balance_velocity_actuator_output_t *actuator_output;

    actuator_input.beam_velocity_deg_s = omega_deg_s;
    actuator_input.motor_position_deg = simple_status.motor_position_deg;
    actuator_input.motor_position_valid =
        simple_motor_feedback_is_fresh(now_ms);
    balance_velocity_actuator_step(&simple_actuator, &actuator_input);
    actuator_output = balance_velocity_actuator_get_output(&simple_actuator);
    simple_status.motor_rpm_requested =
        actuator_output->requested_motor_rpm;
    simple_status.motor_rpm_command = actuator_output->command_motor_rpm;
    simple_status.saturation_flags |= actuator_output->flags;
    simple_desired_rpm = actuator_output->command_motor_rpm;
}

static void simple_update_controller_status(void)
{
    const ball_velocity_controller_output_t *output =
        ball_velocity_controller_get_output(&simple_controller);

    simple_status.position_error_m = output->position_error_m;
    simple_status.velocity_limit_mps = output->velocity_limit_mps;
    simple_status.target_velocity_mps = output->target_velocity_mps;
    simple_status.effective_kv_deg_per_mm =
        output->effective_kv_deg_per_mm;
    simple_status.target_beam_angle_deg = output->target_beam_angle_deg;
    simple_status.beam_angle_error_deg = output->beam_angle_error_deg;
    simple_status.omega_command_deg_s = output->beam_velocity_deg_s;
    simple_status.integral_velocity_mps = output->integral_velocity_mps;
    simple_status.filtered_ball_accel_mps2 =
        output->filtered_acceleration_mps2;
    simple_status.car_feedforward_angle_deg =
        output->vehicle_feedforward_angle_deg;
    simple_status.car_feedforward_scale =
        output->vehicle_feedforward_scale;
    if (0u != (output->flags & BALL_VELOCITY_CONTROL_POSITION_ACTIVE))
        simple_status.flags |= BALANCE_SIMPLE_FLAG_POSITION_ACTIVE;
    else
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_POSITION_ACTIVE);
    simple_status.saturation_flags = (uint16)(output->flags << 8u);
}

static void simple_run_active_control(
    const ball_state_observer_output_t *observer_output,
    uint32 now_ms)
{
    ball_velocity_controller_input_t input;
    float target_position_m = simple_status.target_position_m;
    float measured_beam_angle_deg;

    if (simple_abs(observer_output->position_m) >=
        BALANCE_SIMPLE_BALL_SOFT_EDGE_M)
    {
        target_position_m = 0.0f;
        simple_status.flags |= BALANCE_SIMPLE_FLAG_SOFT_BALL_EDGE;
    }
    else
    {
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_SOFT_BALL_EDGE);
    }
    if (simple_abs(observer_output->position_m) >=
        BALANCE_SIMPLE_BALL_HARD_EDGE_M)
    {
        target_position_m = 0.0f;
        simple_status.flags |= BALANCE_SIMPLE_FLAG_HARD_BALL_EDGE;
    }
    else
    {
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_HARD_BALL_EDGE);
    }
    if (0u == simple_motor_feedback_is_fresh(now_ms))
    {
        ball_velocity_controller_reset(&simple_controller);
        simple_set_state(BALANCE_SIMPLE_SAFE_RETURN, now_ms);
        simple_preempt_with_zero(now_ms);
        return;
    }
    if (0u == balance_linkage_physical_lever_from_motor_deg(
            simple_status.motor_position_deg, &measured_beam_angle_deg))
    {
        ball_velocity_controller_reset(&simple_controller);
        simple_set_state(BALANCE_SIMPLE_SAFE_RETURN, now_ms);
        simple_preempt_with_zero(now_ms);
        return;
    }
    simple_status.measured_beam_angle_deg = measured_beam_angle_deg;

    input.position_m = observer_output->position_m;
    input.velocity_mps = observer_output->velocity_mps;
    input.target_position_m = target_position_m;
    input.measurement_dt_s = simple_measurement_dt_s;
    input.control_dt_s = (float)BALANCE_SIMPLE_CONTROL_PERIOD_MS * 0.001f;
    input.measured_beam_angle_deg = measured_beam_angle_deg;
    input.vehicle_feedforward_angle_deg =
        simple_vehicle_feedforward_angle_deg;
    input.new_measurement = simple_new_measurement;
    input.observer_valid = 1u;
    input.output_saturated = simple_integrator_output_is_limited();
    input.freeze_integral =
        ((BALANCE_SIMPLE_STATIC_LOCK == simple_status.state) ||
         (0u != (simple_status.flags &
                 BALANCE_SIMPLE_FLAG_HARD_BALL_EDGE))) ? 1u : 0u;
    ball_velocity_controller_step(&simple_controller, &input);
    simple_update_controller_status();
    simple_apply_actuator(simple_status.omega_command_deg_s, now_ms);
}

static void simple_update_static_lock(uint32 now_ms)
{
#if (BALANCE_SIMPLE_STATIC_LOCK_ENABLE != 0u)
    if (BALANCE_SIMPLE_ACTIVE == simple_status.state)
    {
        if ((simple_abs(simple_status.position_error_m) <=
             BALANCE_SIMPLE_STATIC_ENTER_POSITION_M) &&
            (simple_abs(simple_status.estimated_velocity_mps) <=
             BALANCE_SIMPLE_STATIC_ENTER_VELOCITY_MPS))
        {
            if (0u == simple_static_stable)
            {
                simple_static_stable = 1u;
                simple_static_stable_start_ms = now_ms;
            }
            else if ((now_ms - simple_static_stable_start_ms) >=
                     BALANCE_SIMPLE_STATIC_ENTER_MS)
            {
                simple_set_state(BALANCE_SIMPLE_STATIC_LOCK, now_ms);
                simple_zero_control_command();
            }
        }
        else
        {
            simple_static_stable = 0u;
        }
    }
    else if ((BALANCE_SIMPLE_STATIC_LOCK == simple_status.state) &&
             ((simple_abs(simple_status.position_error_m) >=
               BALANCE_SIMPLE_STATIC_RELEASE_POSITION_M) ||
              (simple_abs(simple_status.estimated_velocity_mps) >=
               BALANCE_SIMPLE_STATIC_RELEASE_VELOCITY_MPS)))
    {
        ball_velocity_controller_reset(&simple_controller);
        simple_set_state(BALANCE_SIMPLE_ACTIVE, now_ms);
    }
#else
    (void)now_ms;
#endif
}

static void simple_control_step(uint32 now_ms)
{
    ball_state_observer_output_t observer_output;
    float motor_error_deg;
    float omega_deg_s;

    ball_state_observer_get_control_state(
        &simple_observer, now_ms, &observer_output);
    simple_status.estimated_position_m = observer_output.position_m;
    simple_status.estimated_velocity_mps = observer_output.velocity_mps;
    simple_status.position_error_m = observer_output.position_m -
                                     simple_status.target_position_m;
    simple_status.vision_age_ms = observer_output.age_ms;
    if (0u != observer_output.valid)
        simple_status.flags |= BALANCE_SIMPLE_FLAG_OBSERVER_VALID;
    else
        simple_status.flags &=
            (uint16)(~BALANCE_SIMPLE_FLAG_OBSERVER_VALID);

    if (((BALANCE_SIMPLE_ACTIVE == simple_status.state) ||
         (BALANCE_SIMPLE_STATIC_LOCK == simple_status.state)) &&
        (0u == observer_output.valid))
    {
        ball_velocity_controller_reset(&simple_controller);
        simple_set_state(BALANCE_SIMPLE_SAFE_RETURN, now_ms);
        simple_preempt_with_zero(now_ms);
        return;
    }
    if (((BALANCE_SIMPLE_WAIT_VISION == simple_status.state) ||
         (BALANCE_SIMPLE_SAFE_RETURN == simple_status.state)) &&
        (0u != observer_output.valid) &&
        (0u != simple_motor_feedback_is_fresh(now_ms)))
    {
        ball_velocity_controller_reset(&simple_controller);
        simple_set_state(BALANCE_SIMPLE_ACTIVE, now_ms);
    }

    if (BALANCE_SIMPLE_ACTIVE == simple_status.state)
    {
        simple_run_active_control(&observer_output, now_ms);
        simple_update_static_lock(now_ms);
    }
    else if (BALANCE_SIMPLE_STATIC_LOCK == simple_status.state)
    {
        simple_zero_control_command();
        simple_update_static_lock(now_ms);
    }
    else if (BALANCE_SIMPLE_SAFE_RETURN == simple_status.state)
    {
        simple_zero_control_command();
        if ((now_ms - simple_state_start_ms) >=
            BALANCE_SIMPLE_SAFE_RETURN_DELAY_MS)
        {
            if (0u == simple_motor_feedback_is_fresh(now_ms))
            {
                return;
            }
            motor_error_deg =
                simple_level_motor_deg - simple_status.motor_position_deg;
            omega_deg_s = motor_error_deg * 6.0f /
                (BALANCE_SIMPLE_MOTOR_SIGN *
                 BALANCE_SIMPLE_TRANSMISSION_RATIO);
            omega_deg_s = simple_clamp(
                omega_deg_s,
                -BALANCE_SIMPLE_SAFE_RETURN_DEG_S,
                BALANCE_SIMPLE_SAFE_RETURN_DEG_S);
            simple_status.omega_command_deg_s = omega_deg_s;
            simple_apply_actuator(omega_deg_s, now_ms);
        }
    }
    else if (BALANCE_SIMPLE_WAIT_VISION == simple_status.state)
    {
        simple_zero_control_command();
    }
    simple_new_measurement = 0u;
}

static void simple_process_startup(uint32 now_ms)
{
    float error_deg;

    if (BALANCE_SIMPLE_STARTUP_LEVEL != simple_status.state)
    {
        return;
    }
    if ((now_ms - simple_state_start_ms) >
        BALANCE_SIMPLE_STARTUP_TIMEOUT_MS)
    {
        simple_enter_fault(BALANCE_SIMPLE_FAULT_STARTUP_TIMEOUT, now_ms);
        return;
    }
    if ((SIMPLE_START_POWER_WAIT == simple_startup_stage) &&
        ((now_ms - simple_stage_start_ms) >= BALANCE_POWER_WAIT_MS) &&
        (0u == simple_pending_command))
    {
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_ENABLE,
                emm42_set_enabled(SIMPLE_ADDRESS, 0u, 0u), now_ms))
        {
            simple_set_startup_stage(SIMPLE_START_WAIT_DISABLE, now_ms);
        }
    }
    else if ((SIMPLE_START_LOWER_SETTLE == simple_startup_stage) &&
             ((now_ms - simple_stage_start_ms) >=
              BALANCE_LOWER_STOP_SETTLE_MS) &&
             (0u == simple_pending_command))
    {
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_ZERO,
                emm42_set_current_position_zero(SIMPLE_ADDRESS), now_ms))
        {
            simple_set_startup_stage(SIMPLE_START_WAIT_ZERO, now_ms);
        }
    }
    else if ((SIMPLE_START_WAIT_ENABLE == simple_startup_stage) &&
             (0u == simple_pending_command))
    {
        (void)simple_begin_command(
            SIMPLE_COMMAND_ENABLE,
            emm42_set_enabled(SIMPLE_ADDRESS, 1u, 0u), now_ms);
    }
    else if ((SIMPLE_START_WAIT_LEVEL_COMMAND == simple_startup_stage) &&
             (0u == simple_pending_command))
    {
        (void)simple_begin_command(
            SIMPLE_COMMAND_MOVE,
            emm42_move_angle(SIMPLE_ADDRESS, simple_level_motor_deg,
                             BALANCE_EMM42_MOVE_RPM,
                             BALANCE_SIMPLE_EMM42_ACCELERATION,
                             EMM42_POSITION_ABSOLUTE, 0u), now_ms);
    }
    else if (SIMPLE_START_WAIT_LEVEL == simple_startup_stage)
    {
        if (0u != simple_motor_feedback_is_fresh(now_ms))
        {
            error_deg = simple_abs(simple_status.motor_position_deg -
                                   simple_level_motor_deg);
            if (error_deg <= BALANCE_LEVEL_MOTOR_TOLERANCE_DEG)
            {
                if (0u == simple_level_stable)
                {
                    simple_level_stable = 1u;
                    simple_level_stable_start_ms = now_ms;
                }
                else if ((now_ms - simple_level_stable_start_ms) >=
                          BALANCE_LEVEL_SETTLE_MS)
                {
                    simple_zero_control_command();
                    simple_has_sent_rpm = 0u;
                    if (0u != simple_disable_after_startup)
                    {
                        simple_disable_after_startup = 0u;
                        simple_set_state(BALANCE_SIMPLE_DISABLED, now_ms);
                        heartbeat_hw_uart_send_string(
                            "[balance-simple] level; controller disabled\r\n");
                    }
                    else
                    {
                        simple_set_state(BALANCE_SIMPLE_WAIT_VISION, now_ms);
                        heartbeat_hw_uart_send_string(
                            "[balance-simple] level; waiting vision\r\n");
                    }
                }
            }
            else
            {
                simple_level_stable = 0u;
            }
        }
#if (BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE != 0u)
        else if ((now_ms - simple_stage_start_ms) >=
                 BALANCE_SIMPLE_STARTUP_OPEN_LOOP_LEVEL_MS)
        {
            simple_zero_control_command();
            simple_has_sent_rpm = 0u;
            heartbeat_hw_uart_send_string(
                "[balance-simple] level assumed without motor feedback\r\n");
            if (0u != simple_disable_after_startup)
            {
                simple_disable_after_startup = 0u;
                simple_set_state(BALANCE_SIMPLE_DISABLED, now_ms);
                heartbeat_hw_uart_send_string(
                    "[balance-simple] controller disabled after startup\r\n");
            }
            else
            {
                simple_set_state(BALANCE_SIMPLE_WAIT_VISION, now_ms);
            }
        }
#endif
    }
}

static void simple_schedule_motor(uint32 now_ms)
{
    uint8 command_changed;
    uint8 keepalive_due;

    if ((BALANCE_SIMPLE_DISABLED == simple_status.state) ||
        (BALANCE_SIMPLE_FAULT == simple_status.state) ||
        (BALANCE_SIMPLE_STARTUP_LEVEL == simple_status.state))
    {
        return;
    }
    command_changed = ((0u == simple_has_sent_rpm) ||
        (simple_desired_rpm != simple_last_sent_rpm)) ? 1u : 0u;
    keepalive_due = ((now_ms - simple_last_velocity_send_ms) >=
        BALANCE_SIMPLE_VELOCITY_KEEPALIVE_MS) ? 1u : 0u;

    if (0u != command_changed)
    {
        if (0u != simple_pending_command)
        {
            if (0u == simple_command_is_query(simple_pending_command))
            {
                return;
            }
            simple_cancel_pending_query();
        }
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_VELOCITY,
                emm42_run_velocity(SIMPLE_ADDRESS, simple_desired_rpm,
                                   BALANCE_SIMPLE_EMM42_ACCELERATION, 0u),
                now_ms))
        {
            simple_last_sent_rpm = simple_desired_rpm;
            simple_last_velocity_send_ms = now_ms;
            simple_has_sent_rpm = 1u;
        }
        return;
    }

    if (0u != simple_pending_command)
    {
        return;
    }
    if (0u != keepalive_due)
    {
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_VELOCITY,
                emm42_run_velocity(SIMPLE_ADDRESS, simple_desired_rpm,
                                   BALANCE_SIMPLE_EMM42_ACCELERATION, 0u),
                now_ms))
        {
            simple_last_sent_rpm = simple_desired_rpm;
            simple_last_velocity_send_ms = now_ms;
            simple_has_sent_rpm = 1u;
        }
        return;
    }
    if ((now_ms - simple_last_position_query_ms) >=
        BALANCE_SIMPLE_POSITION_QUERY_MS)
    {
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_POSITION,
                emm42_query_position(SIMPLE_ADDRESS), now_ms))
        {
            simple_last_position_query_ms = now_ms;
        }
        return;
    }
    if ((now_ms - simple_last_velocity_query_ms) >=
        BALANCE_SIMPLE_VELOCITY_QUERY_MS)
    {
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_VELOCITY_QUERY,
                emm42_query_velocity(SIMPLE_ADDRESS), now_ms))
        {
            simple_last_velocity_query_ms = now_ms;
        }
    }
}

static void simple_query_startup_position(uint32 now_ms)
{
    if ((BALANCE_SIMPLE_STARTUP_LEVEL == simple_status.state) &&
        (SIMPLE_START_WAIT_LEVEL == simple_startup_stage) &&
        (0u == simple_pending_command) &&
        ((now_ms - simple_last_position_query_ms) >=
         BALANCE_SIMPLE_POSITION_QUERY_MS))
    {
        if (0u != simple_begin_command(
                SIMPLE_COMMAND_POSITION,
                emm42_query_position(SIMPLE_ADDRESS), now_ms))
        {
            simple_last_position_query_ms = now_ms;
        }
    }
}

static uint8 simple_arm_startup(uint32 now_ms)
{
#if (BALANCE_STARTUP_CALIBRATED != 0u)
    if (0u == balance_linkage_motor_from_physical_lever_deg(
            0.0f, &simple_level_motor_deg))
    {
        simple_enter_fault(BALANCE_SIMPLE_FAULT_LINKAGE, now_ms);
        return 0u;
    }

    simple_set_state(BALANCE_SIMPLE_STARTUP_LEVEL, now_ms);
    simple_set_startup_stage(SIMPLE_START_POWER_WAIT, now_ms);
    heartbeat_hw_uart_send_string(
        "[balance-simple] controller=" SIMPLE_CONTROLLER_REVISION
        "; startup armed\r\n");
    return 1u;
#else
    simple_status.fault = BALANCE_SIMPLE_FAULT_NOT_CALIBRATED;
    heartbeat_hw_uart_send_string(
        "[balance-simple] disabled: calibration required\r\n");
    return 0u;
#endif
}

void balance_simple_app_init(void)
{
    ball_state_observer_config_t observer_config;
    ball_velocity_controller_config_t controller_config;
    balance_velocity_actuator_config_t actuator_config;
    uint32 now_ms = heartbeat_get_ms();

    memset(&simple_status, 0, sizeof(simple_status));
    simple_stop_test_mode = 0u;
    observer_config.alpha = BALANCE_SIMPLE_OBSERVER_ALPHA;
    observer_config.beta = BALANCE_SIMPLE_OBSERVER_BETA;
    observer_config.position_limit_m = BALANCE_SIMPLE_VISIBLE_LIMIT_M;
    observer_config.max_implied_speed_mps =
        BALANCE_SIMPLE_MAX_IMPLIED_SPEED_MPS;
    observer_config.max_capture_interval_ms =
        BALANCE_SIMPLE_MAX_CAPTURE_INTERVAL_MS;
    observer_config.valid_timeout_ms = BALANCE_SIMPLE_VISION_TIMEOUT_MS;
    observer_config.transport_latency_ms =
        BALANCE_SIMPLE_VISION_TRANSPORT_MS;
    observer_config.max_prediction_ms =
        BALANCE_SIMPLE_MAX_PREDICTION_MS;
    observer_config.recovery_frames = BALANCE_SIMPLE_RECOVERY_FRAMES;
    ball_state_observer_init(&simple_observer, &observer_config);

    controller_config.position_kp_s_inv = BALANCE_SIMPLE_POSITION_KP_S_INV;
    controller_config.position_ki_s2_inv = BALANCE_SIMPLE_POSITION_KI_S2_INV;
    controller_config.velocity_kv_deg_per_mmps =
        BALANCE_SIMPLE_VELOCITY_KV_DEG_PER_MM;
    controller_config.acceleration_ka_deg_per_mps2 =
        BALANCE_SIMPLE_ACCELERATION_KA;
    controller_config.position_on_m = BALANCE_SIMPLE_POSITION_ON_M;
    controller_config.position_off_m = BALANCE_SIMPLE_POSITION_OFF_M;
    controller_config.max_target_velocity_mps =
        BALANCE_SIMPLE_MAX_TARGET_VELOCITY_MPS;
    controller_config.braking_envelope_mps2 =
        BALANCE_SIMPLE_BRAKING_ENVELOPE_MPS2;
    controller_config.actuator_delay_s =
        BALANCE_SIMPLE_ACTUATOR_DELAY_S;
    controller_config.max_target_beam_angle_deg =
        BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG;
    controller_config.target_beam_angle_slew_deg_s =
        BALANCE_SIMPLE_TARGET_BEAM_ANGLE_SLEW_DEG_S;
    controller_config.beam_angle_kp_s_inv =
        BALANCE_SIMPLE_BEAM_ANGLE_KP_S_INV;
    controller_config.beam_angle_deadband_deg =
        BALANCE_SIMPLE_BEAM_ANGLE_DEADBAND_DEG;
    controller_config.fixed_beam_bias_deg =
        BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG;
    controller_config.max_beam_velocity_deg_s =
        BALANCE_SIMPLE_MAX_BEAM_VELOCITY_DEG_S;
    controller_config.integral_zone_m = BALANCE_SIMPLE_INTEGRAL_ZONE_M;
    controller_config.integral_velocity_limit_mps =
        BALANCE_SIMPLE_INTEGRAL_LIMIT_MPS;
    controller_config.near_position_m = BALANCE_SIMPLE_NEAR_POSITION_M;
    controller_config.near_gain = BALANCE_SIMPLE_NEAR_GAIN;
    controller_config.near_scale_max = BALANCE_SIMPLE_NEAR_SCALE_MAX;
    controller_config.acceleration_filter_alpha =
        BALANCE_SIMPLE_ACCELERATION_FILTER_ALPHA;
    controller_config.vehicle_feedforward_position_cutoff_m =
        BALANCE_SIMPLE_CAR_FF_POSITION_CUTOFF_M;
    ball_velocity_controller_init(&simple_controller, &controller_config);

    actuator_config.motor_sign = BALANCE_SIMPLE_MOTOR_SIGN;
    actuator_config.raising_ratio = BALANCE_SIMPLE_RAISING_RATIO;
    actuator_config.lowering_ratio = BALANCE_SIMPLE_LOWERING_RATIO;
    actuator_config.motor_min_soft_deg = BALANCE_SIMPLE_MOTOR_MIN_SOFT_DEG;
    actuator_config.motor_max_soft_deg = BALANCE_SIMPLE_MOTOR_MAX_SOFT_DEG;
    actuator_config.motor_min_hard_deg = BALANCE_SIMPLE_MOTOR_MIN_HARD_DEG;
    actuator_config.motor_max_hard_deg = BALANCE_SIMPLE_MOTOR_MAX_HARD_DEG;
    actuator_config.max_motor_rpm = BALANCE_SIMPLE_MAX_MOTOR_RPM;
    actuator_config.min_active_rpm = BALANCE_SIMPLE_MIN_ACTIVE_RPM;
    balance_velocity_actuator_init(&simple_actuator, &actuator_config);

    simple_status.state = BALANCE_SIMPLE_DISABLED;
    simple_status.fault = BALANCE_SIMPLE_FAULT_NONE;
    simple_status.vision_age_ms = SIMPLE_INVALID_AGE;
    simple_status.motor_position_age_ms = SIMPLE_INVALID_AGE;
    simple_status.motor_velocity_age_ms = SIMPLE_INVALID_AGE;
#if (BALANCE_SIMPLE_STATIC_LOCK_ENABLE != 0u)
    simple_status.flags |= BALANCE_SIMPLE_FLAG_STATIC_LOCK_ENABLED;
#endif
    simple_rx_frame.length = 0u;
    simple_pending_command = 0u;
    simple_new_measurement = 0u;
    simple_level_stable = 0u;
#if (BALANCE_SIMPLE_STATIC_LOCK_ENABLE != 0u)
    simple_static_stable = 0u;
#endif
    simple_motor_position_valid = 0u;
    simple_motor_velocity_valid = 0u;
    simple_command_errors = 0u;
    simple_disable_after_startup = 0u;
    simple_desired_rpm = 0;
    simple_last_sent_rpm = 0;
    simple_has_sent_rpm = 0u;
    simple_vehicle_feedforward_angle_deg = 0.0f;
    simple_vehicle_filtered_accel_mps2 = 0.0f;
    simple_vehicle_filter_ms = now_ms;
    simple_vehicle_ff_transition_ms = now_ms;
    simple_vehicle_filter_initialized = 0u;
    simple_vehicle_ff_transition_active = 0u;
    simple_vehicle_ff_active = 0u;
    simple_last_capture_ms = 0u;
    simple_last_control_ms = now_ms;
    simple_last_position_query_ms = now_ms;
    simple_last_velocity_query_ms = now_ms;
    simple_last_velocity_send_ms = now_ms;
    simple_motor_position_ms = 0u;
    simple_motor_velocity_ms = 0u;
    emm42_init();
    (void)simple_arm_startup(now_ms);
}

uint8 balance_simple_app_start(void)
{
    if (BALANCE_SIMPLE_FAULT == simple_status.state)
    {
        return 0u;
    }
    if (BALANCE_SIMPLE_DISABLED == simple_status.state)
    {
        balance_simple_app_init();
        return (BALANCE_SIMPLE_FAULT != simple_status.state) ? 1u : 0u;
    }
    if ((BALANCE_SIMPLE_WAIT_VISION == simple_status.state) ||
        (BALANCE_SIMPLE_ACTIVE == simple_status.state) ||
        (BALANCE_SIMPLE_STATIC_LOCK == simple_status.state))
    {
        return balance_simple_app_set_target_position_m(0.0f);
    }
    if (BALANCE_SIMPLE_STARTUP_LEVEL == simple_status.state)
    {
        simple_disable_after_startup = 0u;
        return 1u;
    }
    return 0u;
}

void balance_simple_app_set_stop_test_mode(uint8 enabled)
{
    uint8 requested = (0u != enabled) ? 1u : 0u;

    if (requested == simple_stop_test_mode)
    {
        return;
    }
    simple_stop_test_mode = requested;
    simple_apply_control_mode();
    heartbeat_hw_uart_send_string((0u != requested) ?
        "[balance-simple] stop-test tuning enabled\r\n" :
        "[balance-simple] stop-test tuning disabled\r\n");
}

void balance_simple_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    simple_status.mcu_ms = now_ms;
    simple_drain_motor(now_ms);
    simple_check_pending_timeout(now_ms);
    simple_take_vision_measurement();
    simple_process_startup(now_ms);
    simple_query_startup_position(now_ms);
    if ((now_ms - simple_last_control_ms) >=
        BALANCE_SIMPLE_CONTROL_PERIOD_MS)
    {
        simple_last_control_ms += BALANCE_SIMPLE_CONTROL_PERIOD_MS;
        if ((now_ms - simple_last_control_ms) >=
            BALANCE_SIMPLE_CONTROL_PERIOD_MS)
        {
            simple_last_control_ms = now_ms;
        }
        if ((BALANCE_SIMPLE_STARTUP_LEVEL != simple_status.state) &&
            (BALANCE_SIMPLE_DISABLED != simple_status.state) &&
            (BALANCE_SIMPLE_FAULT != simple_status.state))
        {
            simple_control_step(now_ms);
        }
    }
    simple_schedule_motor(now_ms);
    simple_update_motor_velocity_freshness(now_ms);
    simple_status.motor_position_age_ms =
        (0u != simple_motor_position_valid) ?
        (now_ms - simple_motor_position_ms) : SIMPLE_INVALID_AGE;
    simple_status.motor_velocity_age_ms =
        (0u != simple_motor_velocity_valid) ?
        (now_ms - simple_motor_velocity_ms) : SIMPLE_INVALID_AGE;
    simple_status.emm42_rx_overflow_count =
        (uint16)emm42_get_rx_overflow_count();
}

uint8 balance_simple_app_set_target_position_m(float target_position_m)
{
    if ((simple_abs(target_position_m) >
         BALANCE_SIMPLE_TARGET_LIMIT_M) ||
        ((BALANCE_SIMPLE_ACTIVE != simple_status.state) &&
         (BALANCE_SIMPLE_STATIC_LOCK != simple_status.state) &&
         (BALANCE_SIMPLE_WAIT_VISION != simple_status.state)))
    {
        return 0u;
    }
    simple_status.target_position_m = target_position_m;
    ball_velocity_controller_reset(&simple_controller);
    if (BALANCE_SIMPLE_STATIC_LOCK == simple_status.state)
    {
        simple_set_state(BALANCE_SIMPLE_ACTIVE, heartbeat_get_ms());
    }
    return 1u;
}

void balance_simple_app_set_vehicle_accel_mps2(float accel_mps2, uint8 valid)
{
    float angle_deg = 0.0f;
    float abs_accel;
    uint32 now_ms = heartbeat_get_ms();

    if (0u == valid)
    {
        simple_vehicle_filtered_accel_mps2 = 0.0f;
        simple_vehicle_feedforward_angle_deg = 0.0f;
        simple_vehicle_ff_transition_active = 0u;
        simple_vehicle_ff_active = 0u;
        simple_vehicle_filter_initialized = 0u;
        simple_vehicle_filter_ms = now_ms;
        simple_status.car_accel_mps2 = 0.0f;
        simple_status.car_filtered_accel_mps2 = 0.0f;
        simple_status.car_feedforward_angle_deg = 0.0f;
        simple_status.car_feedforward_scale = 0.0f;
        simple_status.car_accel_valid = 0u;
        simple_status.car_feedforward_active = 0u;
        return;
    }

    accel_mps2 = simple_clamp(
        accel_mps2,
        -BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2,
        BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2);
    if (0u == simple_vehicle_filter_initialized)
    {
        simple_vehicle_filtered_accel_mps2 = accel_mps2;
        simple_vehicle_filter_ms = now_ms;
        simple_vehicle_filter_initialized = 1u;
    }
    else if ((now_ms - simple_vehicle_filter_ms) >=
        BALANCE_SIMPLE_CONTROL_PERIOD_MS)
    {
        simple_vehicle_filtered_accel_mps2 +=
            BALANCE_SIMPLE_CAR_ACCEL_FILTER_ALPHA *
            (accel_mps2 - simple_vehicle_filtered_accel_mps2);
        simple_vehicle_filter_ms = now_ms;
    }
    abs_accel = simple_abs(simple_vehicle_filtered_accel_mps2);
    if (0u == simple_vehicle_ff_active)
    {
        if (abs_accel >= BALANCE_SIMPLE_CAR_FF_ENTER_MPS2)
        {
            if (0u == simple_vehicle_ff_transition_active)
            {
                simple_vehicle_ff_transition_active = 1u;
                simple_vehicle_ff_transition_ms = now_ms;
            }
            else if ((now_ms - simple_vehicle_ff_transition_ms) >=
                     BALANCE_SIMPLE_CAR_FF_ENTER_MS)
            {
                simple_vehicle_ff_active = 1u;
                simple_vehicle_ff_transition_active = 0u;
            }
        }
        else
        {
            simple_vehicle_ff_transition_active = 0u;
        }
    }
    else if (abs_accel <= BALANCE_SIMPLE_CAR_FF_EXIT_MPS2)
    {
        if (0u == simple_vehicle_ff_transition_active)
        {
            simple_vehicle_ff_transition_active = 1u;
            simple_vehicle_ff_transition_ms = now_ms;
        }
        else if ((now_ms - simple_vehicle_ff_transition_ms) >=
                 BALANCE_SIMPLE_CAR_FF_EXIT_MS)
        {
            simple_vehicle_ff_active = 0u;
            simple_vehicle_ff_transition_active = 0u;
        }
    }
    else
    {
        simple_vehicle_ff_transition_active = 0u;
    }

    if (0u != simple_vehicle_ff_active)
    {
        angle_deg = -BALANCE_SIMPLE_CAR_FF_GAIN *
            atan2f(simple_vehicle_filtered_accel_mps2,
                   SIMPLE_GRAVITY_MPS2) * SIMPLE_RAD_TO_DEG;
        angle_deg = simple_clamp(angle_deg,
            -BALANCE_SIMPLE_CAR_FF_MAX_ANGLE_DEG,
            BALANCE_SIMPLE_CAR_FF_MAX_ANGLE_DEG);
    }
    simple_status.car_accel_mps2 = accel_mps2;
    simple_status.car_filtered_accel_mps2 =
        simple_vehicle_filtered_accel_mps2;
    simple_status.car_accel_valid = 1u;
    simple_status.car_feedforward_active = simple_vehicle_ff_active;
    simple_vehicle_feedforward_angle_deg = angle_deg;
}

void balance_simple_app_disable(void)
{
    uint32 now_ms = heartbeat_get_ms();

    if (BALANCE_SIMPLE_STARTUP_LEVEL == simple_status.state)
    {
        simple_disable_after_startup = 1u;
        heartbeat_hw_uart_send_string(
            "[balance-simple] disable deferred until level\r\n");
        return;
    }
    simple_preempt_with_zero(now_ms);
    ball_velocity_controller_reset(&simple_controller);
    balance_simple_app_set_vehicle_accel_mps2(0.0f, 0u);
    simple_set_state(BALANCE_SIMPLE_DISABLED, now_ms);
}

const balance_simple_status_t *balance_simple_app_get_status(void)
{
    return &simple_status;
}
