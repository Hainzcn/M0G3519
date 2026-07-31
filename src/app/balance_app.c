#include "balance_app.h"

#include <math.h>

#include "balance_control.h"
#include "balance_linkage.h"
#include "control_config.h"
#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "vision_link.h"

#define BALANCE_APP_EMM42_ADDRESS        (EMM42_DEFAULT_ADDRESS)
#define BALANCE_APP_ACK_SUCCESS          (0x02u)
#define BALANCE_APP_COMMAND_SET_ZERO      (0x0Au)
#define BALANCE_APP_COMMAND_ENABLE        (0xF3u)
#define BALANCE_APP_COMMAND_MOVE          (0xFDu)
#define BALANCE_APP_COMMAND_POSITION      (0x36u)
#define BALANCE_APP_AGE_INVALID           (0xFFFFFFFFu)

static balance_control_t balance_controller;
static balance_app_status_t balance_status;
static emm42_frame_t balance_rx_frame;
static uint32 balance_state_start_ms;
static uint32 balance_last_control_ms;
static uint32 balance_last_command_ms;
static uint32 balance_last_query_ms;
static uint32 balance_last_accepted_measurement_ms;
static uint32 balance_hard_edge_start_ms;
static uint32 balance_level_tolerance_start_ms;
static float balance_level_motor_deg;
static uint8 balance_pending_command;
static uint32 balance_pending_since_ms;
static uint8 balance_consecutive_command_errors;
static uint8 balance_recovery_valid_frames;
static uint8 balance_has_accepted_measurement;
static uint8 balance_level_move_acked;
static uint8 balance_hard_edge_active;
static uint8 balance_level_tolerance_active;
static uint8 balance_motor_feedback_new;
static uint8 balance_follow_error_count;

static float balance_app_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16 balance_app_u32_to_u16(uint32 value)
{
    return (uint16)((value > 65535u) ? 65535u : value);
}

static void balance_app_set_state(balance_app_state_enum state, uint32 now_ms)
{
    balance_status.state = state;
    balance_state_start_ms = now_ms;
}

static void balance_app_enter_fault(balance_app_fault_enum fault,
                                    uint32 now_ms)
{
    if (BALANCE_APP_FAULT == balance_status.state)
    {
        return;
    }
    balance_status.fault = fault;
    balance_status.flags |= BALANCE_APP_FLAG_FAULT_LATCHED;
    balance_pending_command = 0u;
    balance_status.flags &= (uint8)(~(BALANCE_APP_FLAG_ACTIVE |
                                      BALANCE_APP_FLAG_COMMAND_PENDING));
    balance_app_set_state(BALANCE_APP_FAULT, now_ms);
    (void)emm42_stop(BALANCE_APP_EMM42_ADDRESS, 0u);
    heartbeat_hw_uart_send_string("[balance] latched fault\r\n");
}

static void balance_app_record_command_error(balance_app_fault_enum fault,
                                             uint32 now_ms)
{
    balance_status.command_error_count++;
    if (balance_consecutive_command_errors < 255u)
    {
        balance_consecutive_command_errors++;
    }
    balance_pending_command = 0u;
    balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_COMMAND_PENDING);
    if (balance_consecutive_command_errors >=
        BALANCE_MAX_CONSECUTIVE_COMMAND_ERRORS)
    {
        balance_app_enter_fault(fault, now_ms);
    }
}

static uint8 balance_app_begin_command(uint8 command, uint8 sent,
                                       uint32 now_ms)
{
    if (0u == sent)
    {
        balance_app_record_command_error(BALANCE_FAULT_COMMAND_REJECTED,
                                         now_ms);
        return 0u;
    }
    balance_pending_command = command;
    balance_pending_since_ms = now_ms;
    balance_status.flags |= BALANCE_APP_FLAG_COMMAND_PENDING;
    return 1u;
}

static void balance_app_accept_command(void)
{
    balance_pending_command = 0u;
    balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_COMMAND_PENDING);
    balance_consecutive_command_errors = 0u;
}

static void balance_app_handle_ack(uint8 command, uint8 ack_status,
                                   uint32 now_ms)
{
    if ((command != balance_pending_command) ||
        (BALANCE_APP_ACK_SUCCESS != ack_status))
    {
        if (command == balance_pending_command)
        {
            balance_app_record_command_error(BALANCE_FAULT_COMMAND_REJECTED,
                                             now_ms);
        }
        return;
    }

    balance_app_accept_command();
    if ((BALANCE_APP_SET_REFERENCE == balance_status.state) &&
        (BALANCE_APP_COMMAND_SET_ZERO == command))
    {
        balance_app_set_state(BALANCE_APP_ENABLE, now_ms);
    }
    else if ((BALANCE_APP_ENABLE == balance_status.state) &&
             (BALANCE_APP_COMMAND_ENABLE == command))
    {
        balance_app_set_state(BALANCE_APP_MOVE_LEVEL, now_ms);
    }
    else if ((BALANCE_APP_MOVE_LEVEL == balance_status.state) &&
             (BALANCE_APP_COMMAND_MOVE == command))
    {
        balance_level_move_acked = 1u;
        balance_last_query_ms = now_ms - BALANCE_POSITION_QUERY_PERIOD_MS;
    }
}

static void balance_app_drain_emm42(uint32 now_ms)
{
    uint8 ack_status;
    float position_deg;

    while (0u != emm42_read_frame(&balance_rx_frame))
    {
        if ((0u != balance_pending_command) &&
            (0u != emm42_decode_ack(&balance_rx_frame,
                                    BALANCE_APP_EMM42_ADDRESS,
                                    balance_pending_command, &ack_status)))
        {
            balance_app_handle_ack(balance_pending_command, ack_status,
                                   now_ms);
        }
        else if (0u != emm42_decode_position_deg(
                     &balance_rx_frame, BALANCE_APP_EMM42_ADDRESS,
                     &position_deg))
        {
            balance_status.motor_feedback_deg = position_deg;
            balance_status.flags |= BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID;
            balance_motor_feedback_new = 1u;
            if (BALANCE_APP_COMMAND_POSITION == balance_pending_command)
            {
                balance_app_accept_command();
            }
        }
    }
}

static uint8 balance_app_send_motor_target(float lever_angle_deg,
                                           uint32 now_ms)
{
    float motor_deg;

    if (0u == balance_linkage_relative_motor_deg(
            BALANCE_STARTUP_LEVER_ANGLE_DEG, lever_angle_deg, &motor_deg))
    {
        balance_app_enter_fault(BALANCE_FAULT_LINKAGE_UNREACHABLE, now_ms);
        return 0u;
    }
    balance_status.motor_target_deg = motor_deg;
    balance_last_command_ms = now_ms;
    return balance_app_begin_command(
        BALANCE_APP_COMMAND_MOVE,
        emm42_move_angle(BALANCE_APP_EMM42_ADDRESS, motor_deg,
                         BALANCE_EMM42_MOVE_RPM,
                         BALANCE_EMM42_ACCELERATION,
                         EMM42_POSITION_ABSOLUTE, 0u),
        now_ms);
}

static uint8 balance_app_measurement_acceptable(
    const vision_link_snapshot_t *measurement)
{
    uint8 required = VISION_LINK_FLAG_MEASURED_VALID |
                     VISION_LINK_FLAG_TRACKER_READY |
                     VISION_LINK_FLAG_CALIBRATION_VALID;

    return (((measurement->flags & required) == required) &&
            (measurement->confidence >= BALANCE_MIN_VISION_CONFIDENCE)) ?
        1u : 0u;
}

static void balance_app_control_step(uint32 now_ms)
{
    balance_control_input_t input;
    const balance_control_output_t *output;
    vision_link_snapshot_t measurement;
    vision_link_status_t vision_status;
    uint8 new_measurement = 0u;
    uint32 measurement_age = BALANCE_APP_AGE_INVALID;

    vision_link_get_status(&vision_status);
    if (0u != vision_status.link_online)
    {
        balance_status.flags |= BALANCE_APP_FLAG_LINK_ONLINE;
    }
    else
    {
        balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_LINK_ONLINE);
    }

    if ((0u != vision_link_take_new_valid_measurement(&measurement)) &&
        (0u != balance_app_measurement_acceptable(&measurement)))
    {
        new_measurement = 1u;
        balance_has_accepted_measurement = 1u;
        balance_last_accepted_measurement_ms = measurement.received_ms;
        balance_status.vision_sequence = measurement.sequence;
        balance_status.vision_confidence = measurement.confidence;
        balance_status.flags |= BALANCE_APP_FLAG_MEASUREMENT_ACCEPTED;
        if (balance_recovery_valid_frames < 255u)
        {
            balance_recovery_valid_frames++;
        }
    }

    if (0u != balance_has_accepted_measurement)
    {
        measurement_age = now_ms - balance_last_accepted_measurement_ms;
    }
    balance_status.vision_age_ms = measurement_age;

    input.new_measurement = new_measurement;
    input.measurement_valid =
        ((0u != balance_has_accepted_measurement) &&
         (measurement_age <= BALANCE_VALID_MEASUREMENT_MS)) ? 1u : 0u;
    input.measured_position_m = (0u != new_measurement) ?
        (float)measurement.position_dmm * 0.0001f : 0.0f;
    input.measured_velocity_mps = (0u != new_measurement) ?
        (float)measurement.velocity_mm_s * 0.001f : 0.0f;
    input.measurement_age_ms = measurement_age;
    input.car_accel_mps2 = 0.0f;
    input.dt_s = (float)BALANCE_CONTROL_PERIOD_MS * 0.001f;
    balance_control_step(&balance_controller, &input);
    output = balance_control_get_output(&balance_controller);

    balance_status.control_flags = output->flags;
    balance_status.estimated_position_m = output->estimated_position_m;
    balance_status.estimated_velocity_mps = output->estimated_velocity_mps;
    balance_status.position_error_m = output->position_error_m;
    balance_status.desired_ball_accel_mps2 =
        output->desired_ball_accel_mps2;
    balance_status.lever_angle_deg = output->lever_angle_deg;

    if (0u != (output->flags & BALANCE_CONTROL_FLAG_HARD_EDGE))
    {
        if (0u == balance_hard_edge_active)
        {
            balance_hard_edge_active = 1u;
            balance_hard_edge_start_ms = now_ms;
        }
        else if ((now_ms - balance_hard_edge_start_ms) >=
                 BALANCE_HARD_EDGE_TIMEOUT_MS)
        {
            balance_app_enter_fault(BALANCE_FAULT_BALL_HARD_EDGE, now_ms);
        }
    }
    else
    {
        balance_hard_edge_active = 0u;
    }

    if ((0u != balance_has_accepted_measurement) &&
        ((measurement_age > BALANCE_VALID_MEASUREMENT_MS) ||
         (0u == vision_status.link_online)))
    {
        if (BALANCE_APP_ACTIVE == balance_status.state)
        {
            balance_recovery_valid_frames = 0u;
            balance_app_set_state(BALANCE_APP_RECOVERY, now_ms);
        }
    }
    else if ((BALANCE_APP_RECOVERY == balance_status.state) &&
             (balance_recovery_valid_frames >=
              BALANCE_RECOVERY_VALID_FRAMES))
    {
        balance_app_set_state(BALANCE_APP_ACTIVE, now_ms);
    }
}

static void balance_app_process_startup(uint32 now_ms)
{
    float motor_error;

    if ((BALANCE_APP_POWER_WAIT == balance_status.state) &&
        ((now_ms - balance_state_start_ms) >= BALANCE_POWER_WAIT_MS) &&
        (0u == balance_pending_command))
    {
        if (0u != balance_app_begin_command(
                BALANCE_APP_COMMAND_SET_ZERO,
                emm42_set_current_position_zero(BALANCE_APP_EMM42_ADDRESS),
                now_ms))
        {
            balance_app_set_state(BALANCE_APP_SET_REFERENCE, now_ms);
        }
    }
    else if ((BALANCE_APP_SET_REFERENCE == balance_status.state) &&
             (0u == balance_pending_command))
    {
        (void)balance_app_begin_command(
            BALANCE_APP_COMMAND_SET_ZERO,
            emm42_set_current_position_zero(BALANCE_APP_EMM42_ADDRESS),
            now_ms);
    }
    else if ((BALANCE_APP_ENABLE == balance_status.state) &&
             (0u == balance_pending_command))
    {
        (void)balance_app_begin_command(
            BALANCE_APP_COMMAND_ENABLE,
            emm42_set_enabled(BALANCE_APP_EMM42_ADDRESS, 1u, 0u), now_ms);
    }
    else if ((BALANCE_APP_MOVE_LEVEL == balance_status.state) &&
             (0u == balance_level_move_acked) &&
             (0u == balance_pending_command))
    {
        (void)balance_app_send_motor_target(0.0f, now_ms);
    }
    if (BALANCE_APP_MOVE_LEVEL == balance_status.state)
    {
        if ((now_ms - balance_state_start_ms) >
            BALANCE_MOVE_LEVEL_TIMEOUT_MS)
        {
            balance_app_enter_fault(BALANCE_FAULT_MOVE_LEVEL_TIMEOUT, now_ms);
            return;
        }
        if (0u != (balance_status.flags &
                   BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID))
        {
            motor_error = balance_app_abs(balance_status.motor_feedback_deg -
                                          balance_level_motor_deg);
            if (motor_error <= BALANCE_LEVEL_MOTOR_TOLERANCE_DEG)
            {
                if (0u == balance_level_tolerance_active)
                {
                    balance_level_tolerance_active = 1u;
                    balance_level_tolerance_start_ms = now_ms;
                }
                else if ((now_ms - balance_level_tolerance_start_ms) >=
                         BALANCE_LEVEL_SETTLE_MS)
                {
                    balance_status.flags |= BALANCE_APP_FLAG_ACTIVE;
                    balance_app_set_state(BALANCE_APP_ACTIVE, now_ms);
                    balance_last_control_ms = now_ms;
                    balance_last_command_ms = now_ms;
                    balance_last_query_ms = now_ms;
                }
            }
            else
            {
                balance_level_tolerance_active = 0u;
            }
        }
    }
}

static void balance_app_process_active(uint32 now_ms)
{
    float follow_error;

    if ((now_ms - balance_last_control_ms) >= BALANCE_CONTROL_PERIOD_MS)
    {
        balance_last_control_ms += BALANCE_CONTROL_PERIOD_MS;
        if ((now_ms - balance_last_control_ms) >= BALANCE_CONTROL_PERIOD_MS)
        {
            balance_last_control_ms = now_ms;
        }
        balance_app_control_step(now_ms);
    }

    if ((BALANCE_APP_FAULT == balance_status.state) ||
        (0u != balance_pending_command))
    {
        return;
    }
    if ((now_ms - balance_last_command_ms) >= BALANCE_COMMAND_PERIOD_MS)
    {
        (void)balance_app_send_motor_target(balance_status.lever_angle_deg,
                                            now_ms);
    }

    if (0u != balance_motor_feedback_new)
    {
        balance_motor_feedback_new = 0u;
        follow_error = balance_app_abs(balance_status.motor_feedback_deg -
                                       balance_status.motor_target_deg);
        if (follow_error > BALANCE_MOTOR_FOLLOW_ERROR_DEG)
        {
            balance_status.command_error_count++;
            if (balance_follow_error_count < 255u)
            {
                balance_follow_error_count++;
            }
            if (balance_follow_error_count >=
                BALANCE_MAX_CONSECUTIVE_COMMAND_ERRORS)
            {
                balance_app_enter_fault(
                    BALANCE_FAULT_MOTOR_FOLLOW_ERROR, now_ms);
            }
        }
        else
        {
            balance_follow_error_count = 0u;
        }
    }
}

static void balance_app_query_position_if_due(uint32 now_ms)
{
    if ((BALANCE_APP_FAULT == balance_status.state) ||
        (0u != balance_pending_command) ||
        ((now_ms - balance_last_query_ms) <
         BALANCE_POSITION_QUERY_PERIOD_MS))
    {
        return;
    }

    balance_last_query_ms = now_ms;
    (void)balance_app_begin_command(
        BALANCE_APP_COMMAND_POSITION,
        emm42_query_position(BALANCE_APP_EMM42_ADDRESS), now_ms);
}

void balance_app_init(void)
{
    balance_control_config_t config;
    uint32 now_ms = heartbeat_get_ms();

    config.kp = BALANCE_KP;
    config.kd = BALANCE_KD;
    config.position_correction_gain = BALANCE_ESTIMATOR_POSITION_GAIN;
    config.velocity_correction_gain = BALANCE_ESTIMATOR_VELOCITY_GAIN;
    config.max_ball_accel_mps2 = BALANCE_MAX_BALL_ACCEL_MPS2;
    config.max_lever_angle_deg = BALANCE_MAX_LEVER_ANGLE_DEG;
    config.degraded_lever_angle_deg = BALANCE_DEGRADED_LEVER_ANGLE_DEG;
    config.max_lever_rate_deg_s = BALANCE_MAX_LEVER_RATE_DEG_S;
    config.edge_position_m = BALANCE_EDGE_POSITION_M;
    config.hard_edge_position_m = BALANCE_HARD_EDGE_POSITION_M;
    config.fresh_measurement_ms = BALANCE_FRESH_MEASUREMENT_MS;
    config.valid_measurement_ms = BALANCE_VALID_MEASUREMENT_MS;
    balance_control_init(&balance_controller, &config);

    balance_status.state = BALANCE_APP_UNCONFIGURED;
    balance_status.fault = BALANCE_FAULT_NONE;
    balance_status.flags = 0u;
    balance_status.control_flags = 0u;
    balance_status.vision_confidence = 0u;
    balance_status.vision_sequence = 0u;
    balance_status.vision_age_ms = BALANCE_APP_AGE_INVALID;
    balance_status.estimated_position_m = 0.0f;
    balance_status.estimated_velocity_mps = 0.0f;
    balance_status.position_error_m = 0.0f;
    balance_status.desired_ball_accel_mps2 = 0.0f;
    balance_status.lever_angle_deg = 0.0f;
    balance_status.motor_target_deg = 0.0f;
    balance_status.motor_feedback_deg = 0.0f;
    balance_status.command_error_count = 0u;
    balance_status.emm42_rx_overflow_count = 0u;
    balance_rx_frame.length = 0u;
    balance_pending_command = 0u;
    balance_consecutive_command_errors = 0u;
    balance_recovery_valid_frames = 0u;
    balance_has_accepted_measurement = 0u;
    balance_level_move_acked = 0u;
    balance_hard_edge_active = 0u;
    balance_level_tolerance_active = 0u;
    balance_motor_feedback_new = 0u;
    balance_follow_error_count = 0u;
    balance_state_start_ms = now_ms;
    balance_last_control_ms = now_ms;
    balance_last_command_ms = now_ms;
    balance_last_query_ms = now_ms;
    balance_last_accepted_measurement_ms = 0u;
    balance_hard_edge_start_ms = 0u;
    balance_level_tolerance_start_ms = 0u;

    emm42_init();
#if (BALANCE_STARTUP_CALIBRATED != 0u)
    if (0u == balance_linkage_relative_motor_deg(
            BALANCE_STARTUP_LEVER_ANGLE_DEG, 0.0f,
            &balance_level_motor_deg))
    {
        balance_app_enter_fault(BALANCE_FAULT_LINKAGE_UNREACHABLE, now_ms);
    }
    else
    {
        balance_app_set_state(BALANCE_APP_POWER_WAIT, now_ms);
        heartbeat_hw_uart_send_string(
            "[balance] calibrated startup armed\r\n");
    }
#else
    balance_level_motor_deg = 0.0f;
    heartbeat_hw_uart_send_string(
        "[balance] UNCONFIGURED: startup angle not calibrated\r\n");
#endif
}

void balance_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    balance_app_drain_emm42(now_ms);
    balance_status.emm42_rx_overflow_count =
        balance_app_u32_to_u16(emm42_get_rx_overflow_count());
    if ((0u != balance_pending_command) &&
        ((now_ms - balance_pending_since_ms) > BALANCE_COMMAND_TIMEOUT_MS))
    {
        balance_app_record_command_error(BALANCE_FAULT_COMMAND_TIMEOUT,
                                         now_ms);
    }

    if ((BALANCE_APP_POWER_WAIT == balance_status.state) ||
        (BALANCE_APP_SET_REFERENCE == balance_status.state) ||
        (BALANCE_APP_ENABLE == balance_status.state) ||
        (BALANCE_APP_MOVE_LEVEL == balance_status.state))
    {
        balance_app_process_startup(now_ms);
    }
    else if ((BALANCE_APP_ACTIVE == balance_status.state) ||
             (BALANCE_APP_RECOVERY == balance_status.state))
    {
        balance_app_process_active(now_ms);
    }
    balance_app_query_position_if_due(now_ms);
}

const balance_app_status_t *balance_app_get_status(void)
{
    return &balance_status;
}
