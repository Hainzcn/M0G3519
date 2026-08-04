#include "balance_app.h"

#include <math.h>
#include <stdio.h>

#include "balance_control.h"
#include "balance_actuator_trajectory.h"
#include "balance_linkage.h"
#include "ball_motion_profile.h"
#include "control_config.h"
#include "control_config_legacy.h"
#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "motor_app.h"
#include "vision_link.h"
#include "wheel_speed_control.h"

#define BALANCE_APP_EMM42_ADDRESS        (EMM42_DEFAULT_ADDRESS)
#define BALANCE_APP_ACK_SUCCESS          (0x02u)
#define BALANCE_APP_COMMAND_SET_ZERO      (0x0Au)
#define BALANCE_APP_COMMAND_ENABLE        (0xF3u)
#define BALANCE_APP_COMMAND_MOVE          (0xFDu)
#define BALANCE_APP_COMMAND_POSITION      (0x36u)
#define BALANCE_APP_AGE_INVALID           (0xFFFFFFFFu)

static balance_control_t balance_controller;
static balance_actuator_trajectory_t balance_actuator_trajectory;
static ball_motion_profile_t balance_profile;
static balance_app_status_t balance_status;
static emm42_frame_t balance_rx_frame;
static uint32 balance_state_start_ms;
static uint32 balance_last_control_ms;
static uint32 balance_last_outer_control_ms;
static uint32 balance_last_command_ms;
static uint32 balance_last_query_ms;
static uint32 balance_last_accepted_measurement_ms;
static uint32 balance_last_measurement_latency_ms;
static uint32 balance_previous_measurement_received_ms;
static uint32 balance_hard_edge_start_ms;
static uint32 balance_level_tolerance_start_ms;
static float balance_level_motor_deg;
static uint8 balance_pending_command;
static uint32 balance_pending_since_ms;
static uint8 balance_consecutive_command_errors;
static uint8 balance_consecutive_position_query_errors;
static uint8 balance_recovery_valid_frames;
static uint8 balance_has_accepted_measurement;
static uint8 balance_level_move_acked;
static uint8 balance_hard_edge_active;
static uint8 balance_level_tolerance_active;
static uint8 balance_motor_feedback_new;
static uint8 balance_follow_error_active;
static uint32 balance_follow_error_start_ms;
static uint8 balance_has_seen_vision_snapshot;
static uint16 balance_last_seen_vision_sequence;
static uint16 balance_last_seen_vision_boot_id;
static float balance_last_sent_lever_deg;
static uint8 balance_has_sent_lever_target;
static uint8 balance_actuator_command_pending;
static float balance_recovery_candidate_position_m;
static uint8 balance_recovery_candidate_valid;
static uint32 balance_sequence_start_ms;
static uint32 balance_sequence_settle_start_ms;
static uint8 balance_sequence_settle_active;
static uint8 balance_sequence_start_pending;
static uint32 balance_last_car_feedforward_debug_ms;

static void balance_app_begin_sequence(uint32 now_ms);

static float balance_app_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16 balance_app_u32_to_u16(uint32 value)
{
    return (uint16)((value > 65535u) ? 65535u : value);
}

static void balance_app_set_sequence_state(
    balance_app_sequence_state_enum state)
{
    balance_status.sequence_state = state;
    if ((BALANCE_SEQUENCE_TO_POSITIVE == state) ||
        (BALANCE_SEQUENCE_TO_NEGATIVE == state))
    {
        balance_status.flags |= BALANCE_APP_FLAG_SEQUENCE_ACTIVE;
    }
    else
    {
        balance_status.flags &=
            (uint8)(~BALANCE_APP_FLAG_SEQUENCE_ACTIVE);
    }
    balance_sequence_settle_active = 0u;
}

static void balance_app_begin_negative_sequence_leg(uint32 now_ms)
{
    ball_motion_profile_set_target(
        &balance_profile, BALANCE_SEQUENCE_NEGATIVE_TARGET_M);
    balance_sequence_start_ms = now_ms;
    balance_status.sequence_elapsed_ms = 0u;
    balance_app_set_sequence_state(BALANCE_SEQUENCE_TO_NEGATIVE);
}

static void balance_app_abort_sequence(
    balance_app_sequence_state_enum state)
{
    balance_sequence_start_pending = 0u;
    ball_motion_profile_set_target(&balance_profile, 0.0f);
    balance_app_set_sequence_state(state);
}

static float balance_app_clamp(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

static uint8 balance_app_get_car_feedforward_accel(
    uint32 now_ms, float *imu_accel_mps2, uint32 *imu_age_ms,
    float *feedforward_accel_mps2)
{
    imu_snapshot_t imu;

    imu_get_snapshot(&imu);
    *imu_accel_mps2 = imu.accel.ax;
    *imu_age_ms = BALANCE_APP_AGE_INVALID;
    *feedforward_accel_mps2 = 0.0f;
    if (0u == (imu.flags & IMU_FLAG_ACCEL))
    {
        return 0u;
    }

    *imu_age_ms = now_ms - imu.accel_time_ms;
    if (*imu_age_ms > BALANCE_CAR_IMU_MAX_AGE_MS)
    {
        return 0u;
    }

    /* ax is the effective longitudinal acceleration in the car frame. */
    *feedforward_accel_mps2 = balance_app_clamp(
        (*imu_accel_mps2 - BALANCE_CAR_IMU_ACCEL_OFFSET_MPS2) *
            BALANCE_CAR_IMU_ACCEL_GAIN * BALANCE_CAR_IMU_ACCEL_SIGN,
        -BALANCE_CAR_IMU_ACCEL_LIMIT_MPS2,
        BALANCE_CAR_IMU_ACCEL_LIMIT_MPS2);
    return 1u;
}

static void balance_app_send_car_feedforward_debug(uint32 now_ms)
{
    char message[192];

    if ((now_ms - balance_last_car_feedforward_debug_ms) <
        BALANCE_CAR_FEEDFORWARD_DEBUG_PERIOD_MS)
    {
        return;
    }
    balance_last_car_feedforward_debug_ms = now_ms;
    snprintf(message, sizeof(message),
        "[car-ff] imu=%.3f,age=%lu,enc=%.3f,ff=%.3f,sync=%.2f,raw=%.2f,m=%.2f,ok=%u\r\n",
        (double)balance_status.car_imu_accel_mps2,
        (unsigned long)balance_status.car_imu_accel_age_ms,
        (double)balance_status.car_encoder_accel_mps2,
        (double)balance_status.car_feedforward_accel_mps2,
        (double)balance_status.car_sync_lever_angle_deg,
        (double)balance_status.raw_lever_angle_deg,
        (double)balance_status.motor_target_deg,
        (unsigned int)balance_status.car_imu_accel_valid);
    heartbeat_hw_uart_send_string(message);
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
    balance_app_abort_sequence(BALANCE_SEQUENCE_CANCELED);
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

static void balance_app_record_position_query_error(uint32 now_ms)
{
    balance_status.command_error_count++;
    if (balance_consecutive_position_query_errors < 255u)
    {
        balance_consecutive_position_query_errors++;
    }
    balance_pending_command = 0u;
    balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_COMMAND_PENDING);
    if (balance_consecutive_position_query_errors >=
        BALANCE_MAX_CONSECUTIVE_POSITION_QUERY_ERRORS)
    {
        balance_app_enter_fault(BALANCE_FAULT_COMMAND_TIMEOUT, now_ms);
    }
}

static uint8 balance_app_begin_command(uint8 command, uint8 sent,
                                       uint32 now_ms)
{
    if (0u == sent)
    {
        if (BALANCE_APP_COMMAND_POSITION == command)
            balance_app_record_position_query_error(now_ms);
        else
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
    if ((BALANCE_APP_DISABLE == balance_status.state) &&
        (BALANCE_APP_COMMAND_ENABLE == command))
    {
        balance_app_set_state(BALANCE_APP_WAIT_LOWER_STOP, now_ms);
        heartbeat_hw_uart_send_string(
            "[balance] disabled; wait for lower stop\r\n");
    }
    else if ((BALANCE_APP_SET_REFERENCE == balance_status.state) &&
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
            balance_consecutive_position_query_errors = 0u;
            balance_status.motor_feedback_deg = position_deg;
            balance_status.flags |= BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID;
            if (0u != balance_linkage_physical_lever_from_motor_deg(
                    position_deg,
                    &balance_status.actual_lever_angle_deg))
            {
                balance_status.actual_lever_angle_deg *=
                    (float)BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN;
                balance_status.flags |=
                    BALANCE_APP_FLAG_LEVER_FEEDBACK_VALID;
            }
            else
            {
                balance_status.flags &=
                    (uint8)(~BALANCE_APP_FLAG_LEVER_FEEDBACK_VALID);
            }
            balance_motor_feedback_new = 1u;
            if (BALANCE_APP_COMMAND_POSITION == balance_pending_command)
            {
                balance_app_accept_command();
            }
        }
    }
}

static uint8 balance_app_send_motor_target(float lever_angle_deg,
                                           uint16 move_rpm,
                                           uint32 now_ms)
{
    float motor_deg;

    if (0u == balance_linkage_motor_from_physical_lever_deg(
            (float)BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN * lever_angle_deg,
            &motor_deg))
    {
        balance_app_enter_fault(BALANCE_FAULT_LINKAGE_UNREACHABLE, now_ms);
        return 0u;
    }
    balance_status.motor_target_deg = motor_deg;
    balance_last_command_ms = now_ms;
    if (0u == balance_app_begin_command(
        BALANCE_APP_COMMAND_MOVE,
        emm42_move_angle(BALANCE_APP_EMM42_ADDRESS, motor_deg,
                         move_rpm,
                         BALANCE_EMM42_ACCELERATION,
                         EMM42_POSITION_ABSOLUTE, 0u),
        now_ms))
    {
        return 0u;
    }
    balance_last_sent_lever_deg = lever_angle_deg;
    balance_has_sent_lever_target = 1u;
    balance_actuator_command_pending = 1u;
    return 1u;
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

static uint8 balance_app_recovery_measurement_consistent(
    const vision_link_snapshot_t *measurement)
{
    float position_m;

    if (BALANCE_APP_ACTIVE == balance_status.state)
    {
        return 1u;
    }
    position_m = (float)measurement->position_dmm * 0.0001f +
        BALANCE_VISION_POSITION_OFFSET_M;
    if (0u == balance_recovery_candidate_valid)
    {
        balance_recovery_candidate_position_m = position_m;
        balance_recovery_candidate_valid = 1u;
        return 1u;
    }
    if (balance_app_abs(position_m -
                        balance_recovery_candidate_position_m) >
        BALANCE_RECOVERY_MAX_POSITION_STEP_M)
    {
        balance_recovery_candidate_position_m = position_m;
        balance_recovery_valid_frames = 0u;
        return 0u;
    }
    balance_recovery_candidate_position_m = position_m;
    return 1u;
}

static void balance_app_control_step(uint32 now_ms, uint8 update_output)
{
    balance_control_input_t input;
    const balance_control_output_t *output;
    vision_link_snapshot_t measurement;
    vision_link_status_t vision_status;
    uint8 new_measurement = 0u;
    uint8 new_snapshot = 0u;
    uint32 measurement_age = BALANCE_APP_AGE_INVALID;
    uint32 measurement_latency_ms;
    float measurement_interval_s = 0.0f;
    const ball_motion_profile_output_t *profile_output;
    const balance_actuator_trajectory_output_t *actuator_output;
    const wheel_speed_control_status_t *wheel_status;

    vision_link_get_status(&vision_status);
    if (0u != vision_status.link_online)
    {
        balance_status.flags |= BALANCE_APP_FLAG_LINK_ONLINE;
    }
    else
    {
        balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_LINK_ONLINE);
    }

    if (0u != vision_link_get_latest_snapshot(&measurement))
    {
        new_snapshot = ((0u == balance_has_seen_vision_snapshot) ||
                        (measurement.boot_id !=
                         balance_last_seen_vision_boot_id) ||
                        (measurement.sequence !=
                         balance_last_seen_vision_sequence)) ? 1u : 0u;
        if (0u != new_snapshot)
        {
            balance_has_seen_vision_snapshot = 1u;
            balance_last_seen_vision_boot_id = measurement.boot_id;
            balance_last_seen_vision_sequence = measurement.sequence;
            balance_status.vision_raw_flags = measurement.flags;
            balance_status.vision_raw_confidence = measurement.confidence;
            balance_status.vision_raw_sequence = measurement.sequence;
            balance_status.vision_raw_position_dmm =
                measurement.position_dmm;
            balance_status.vision_raw_velocity_mm_s =
                measurement.velocity_mm_s;

            if ((0u != balance_app_measurement_acceptable(&measurement)) &&
                (0u != balance_app_recovery_measurement_consistent(
                    &measurement)))
            {
                new_measurement = 1u;
                if (0u != balance_has_accepted_measurement)
                {
                    measurement_interval_s =
                        (float)(measurement.received_ms -
                                balance_previous_measurement_received_ms) *
                        0.001f;
                }
                balance_has_accepted_measurement = 1u;
                balance_previous_measurement_received_ms =
                    measurement.received_ms;
                balance_last_accepted_measurement_ms =
                    measurement.received_ms;
                measurement_latency_ms =
                    (uint32)measurement.processing_ms +
                    BALANCE_VISION_TRANSPORT_LATENCY_MS;
                if (measurement_latency_ms >
                    BALANCE_VISION_MAX_COMPENSATION_MS)
                {
                    measurement_latency_ms =
                        BALANCE_VISION_MAX_COMPENSATION_MS;
                }
                balance_last_measurement_latency_ms =
                    measurement_latency_ms;
                balance_status.vision_sequence = measurement.sequence;
                balance_status.vision_confidence = measurement.confidence;
                balance_status.flags |=
                    BALANCE_APP_FLAG_MEASUREMENT_ACCEPTED;
                if (balance_recovery_valid_frames < 255u)
                {
                    balance_recovery_valid_frames++;
                }
            }
            else
            {
                balance_recovery_valid_frames = 0u;
                balance_status.flags &=
                    (uint8)(~BALANCE_APP_FLAG_MEASUREMENT_ACCEPTED);
            }
        }
    }

    if (0u != balance_has_accepted_measurement)
    {
        measurement_age = now_ms - balance_last_accepted_measurement_ms;
        if (measurement_age <=
            (BALANCE_APP_AGE_INVALID -
             balance_last_measurement_latency_ms))
        {
            measurement_age += balance_last_measurement_latency_ms;
        }
        else
        {
            measurement_age = BALANCE_APP_AGE_INVALID;
        }
    }
    balance_status.vision_age_ms = measurement_age;

    input.new_measurement = new_measurement;
    input.measurement_valid =
        ((0u != balance_has_accepted_measurement) &&
         (measurement_age <= BALANCE_VALID_MEASUREMENT_MS)) ? 1u : 0u;
    input.measured_position_m = (0u != new_measurement) ?
        (float)measurement.position_dmm * 0.0001f +
        BALANCE_VISION_POSITION_OFFSET_M +
        (float)measurement.velocity_mm_s * 0.001f *
            (float)balance_last_measurement_latency_ms * 0.001f : 0.0f;
    input.measured_velocity_mps = (0u != new_measurement) ?
        (float)measurement.velocity_mm_s * 0.001f : 0.0f;
    input.measurement_interval_s = measurement_interval_s;
    input.measurement_age_ms = measurement_age;
    output = balance_control_get_output(&balance_controller);
    if (BALANCE_APP_ACTIVE == balance_status.state)
    {
        ball_motion_profile_step(
            &balance_profile,
            (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f);
    }
    profile_output = ball_motion_profile_get_output(&balance_profile);
    input.reference_position_m = profile_output->position_m;
    input.reference_velocity_mps = profile_output->velocity_mps;
    input.target_position_m = profile_output->target_position_m;
    input.feedforward_accel_mps2 =
        profile_output->feedforward_accel_mps2;
    input.reference_holding =
        (BALL_MOTION_PHASE_HOLD == profile_output->phase) ? 1u : 0u;
    wheel_status = wheel_speed_control_get_status();
    balance_status.car_imu_accel_valid =
        balance_app_get_car_feedforward_accel(
            now_ms,
            &balance_status.car_imu_accel_mps2,
            &balance_status.car_imu_accel_age_ms,
            &input.car_accel_mps2);
    /* Chassis acceleration is relevant only while chassis control is active. */
    if (MOTOR_APP_MODE_DISABLED == motor_app_get_mode())
    {
        input.car_accel_mps2 = 0.0f;
    }
    /* Position polling is 10 Hz; only a newly decoded sample is model-fresh. */
    input.actual_lever_valid = balance_motor_feedback_new;
    input.actual_lever_angle_deg = balance_status.actual_lever_angle_deg;
    actuator_output = balance_actuator_trajectory_get_output(
        &balance_actuator_trajectory);
    input.actuator_command_updated = balance_actuator_command_pending;
    input.actuator_command_angle_deg = (0u != balance_has_sent_lever_target) ?
        balance_last_sent_lever_deg : 0.0f;
    balance_actuator_command_pending = 0u;
    input.update_control_output = update_output;
    input.dt_s = (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f;
    balance_control_step(&balance_controller, &input);
    output = balance_control_get_output(&balance_controller);

    if (0u != update_output)
    {
        balance_actuator_trajectory_step(
            &balance_actuator_trajectory,
            (BALANCE_APP_ACTIVE == balance_status.state) ?
                output->lever_angle_deg : 0.0f,
            (float)BALANCE_OUTER_CONTROL_PERIOD_MS * 0.001f);
        actuator_output = balance_actuator_trajectory_get_output(
            &balance_actuator_trajectory);
    }
    balance_status.control_flags = output->flags;
    if (0u != actuator_output->saturated)
    {
        balance_status.control_flags |= BALANCE_CONTROL_FLAG_SLEW_SATURATED;
    }
    balance_status.estimated_position_m = output->estimated_position_m;
    balance_status.estimated_velocity_mps = output->estimated_velocity_mps;
    balance_status.predicted_position_m = output->predicted_position_m;
    balance_status.predicted_velocity_mps = output->predicted_velocity_mps;
    balance_status.target_position_m = profile_output->target_position_m;
    balance_status.reference_position_m = profile_output->position_m;
    balance_status.reference_velocity_mps = profile_output->velocity_mps;
    balance_status.reference_accel_mps2 = profile_output->accel_mps2;
    balance_status.motion_phase = profile_output->phase;
    balance_status.position_error_m = output->position_error_m;
    balance_status.velocity_command_mps = output->velocity_command_mps;
    balance_status.velocity_limit_mps = output->velocity_limit_mps;
    balance_status.brake_distance_m = output->brake_distance_m;
    balance_status.feedforward_accel_mps2 = output->feedforward_accel_mps2;
    balance_status.feedback_accel_mps2 = output->feedback_accel_mps2;
    balance_status.desired_ball_accel_mps2 =
        output->desired_ball_accel_mps2;
    balance_status.car_encoder_speed_mps = (NULL != wheel_status) ?
        wheel_status->measured_speed_mps : 0.0f;
    balance_status.car_encoder_accel_mps2 = (NULL != wheel_status) ?
        wheel_status->measured_accel_mps2 : 0.0f;
    balance_status.car_feedforward_accel_mps2 = input.car_accel_mps2;
    balance_status.car_sync_lever_angle_deg =
        balance_control_vehicle_sync_lever_deg(input.car_accel_mps2);
    balance_status.raw_lever_angle_deg = output->lever_angle_deg;
    balance_status.lever_angle_deg = actuator_output->angle_deg;
    balance_status.control_phase = output->phase;
    balance_status.friction_mode = output->friction_mode;

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

    balance_app_send_car_feedforward_debug(now_ms);

    if ((0u != balance_has_accepted_measurement) &&
        ((measurement_age > BALANCE_VALID_MEASUREMENT_MS) ||
         (0u == vision_status.link_online)))
    {
        balance_status.flags &=
            (uint8)(~BALANCE_APP_FLAG_MEASUREMENT_ACCEPTED);
        if (BALANCE_APP_ACTIVE == balance_status.state)
        {
            float recovery_start_angle =
                (0u != input.actual_lever_valid) ?
                    input.actual_lever_angle_deg : actuator_output->angle_deg;
            balance_recovery_valid_frames = 0u;
            balance_recovery_candidate_valid = 0u;
            balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_ACTIVE);
            balance_app_abort_sequence(BALANCE_SEQUENCE_CANCELED);
            balance_control_reset(&balance_controller);
            balance_actuator_trajectory_reset(
                &balance_actuator_trajectory, recovery_start_angle);
            balance_actuator_trajectory_step(
                &balance_actuator_trajectory, 0.0f,
                (float)BALANCE_OUTER_CONTROL_PERIOD_MS * 0.001f);
            balance_actuator_command_pending = 0u;
            balance_app_set_state(BALANCE_APP_RECOVERY, now_ms);
        }
    }
    else if (((BALANCE_APP_RECOVERY == balance_status.state) ||
              (BALANCE_APP_WAIT_VISION == balance_status.state)) &&
             (balance_recovery_valid_frames >=
              BALANCE_RECOVERY_VALID_FRAMES))
    {
        float restart_angle = (0u != input.actual_lever_valid) ?
            input.actual_lever_angle_deg : actuator_output->angle_deg;

        balance_control_reset(&balance_controller);
        balance_actuator_trajectory_reset(
            &balance_actuator_trajectory, restart_angle);
        input.actuator_command_updated = 1u;
        input.actuator_command_angle_deg =
            (0u != balance_has_sent_lever_target) ?
                balance_last_sent_lever_deg : restart_angle;
        input.actual_lever_valid = 1u;
        input.actual_lever_angle_deg = restart_angle;
        input.update_control_output = 0u;
        balance_control_step(&balance_controller, &input);
        output = balance_control_get_output(&balance_controller);
        balance_status.flags |= BALANCE_APP_FLAG_ACTIVE;
        ball_motion_profile_reset(
            &balance_profile,
            output->estimated_position_m,
            output->estimated_velocity_mps);
        ball_motion_profile_set_target(&balance_profile, 0.0f);
        balance_recovery_candidate_valid = 0u;
        balance_actuator_command_pending = 0u;
        balance_app_set_state(BALANCE_APP_ACTIVE, now_ms);
        heartbeat_hw_uart_send_string("[balance] active\r\n");
    }
}

static void balance_app_update_sequence(uint32 now_ms)
{
    uint8 arrived;

    if ((BALANCE_SEQUENCE_TO_POSITIVE != balance_status.sequence_state) &&
        (BALANCE_SEQUENCE_TO_NEGATIVE != balance_status.sequence_state))
    {
        return;
    }
    balance_status.sequence_elapsed_ms = now_ms - balance_sequence_start_ms;
    if (balance_status.sequence_elapsed_ms >= BALANCE_SEQUENCE_TIMEOUT_MS)
    {
        if (BALANCE_SEQUENCE_TO_POSITIVE ==
            balance_status.sequence_state)
        {
            balance_app_begin_negative_sequence_leg(now_ms);
            heartbeat_hw_uart_send_string(
                "[balance] positive leg timeout; continuing\r\n");
        }
        else
        {
            balance_app_abort_sequence(BALANCE_SEQUENCE_TIMEOUT);
            heartbeat_hw_uart_send_string(
                "[balance] negative leg timeout\r\n");
        }
        return;
    }

    arrived = ((balance_app_abs(balance_status.target_position_m -
                                balance_status.estimated_position_m) <=
                BALANCE_SEQUENCE_POSITION_TOLERANCE_M) &&
        (balance_app_abs(balance_status.estimated_velocity_mps) <=
         BALANCE_SEQUENCE_VELOCITY_TOLERANCE_MPS)) ? 1u : 0u;
    if (0u == arrived)
    {
        balance_sequence_settle_active = 0u;
        return;
    }
    if (0u == balance_sequence_settle_active)
    {
        balance_sequence_settle_active = 1u;
        balance_sequence_settle_start_ms = now_ms;
        return;
    }
    if ((now_ms - balance_sequence_settle_start_ms) <
        BALANCE_SEQUENCE_SETTLE_MS)
    {
        return;
    }

    if (BALANCE_SEQUENCE_TO_POSITIVE == balance_status.sequence_state)
    {
        balance_app_begin_negative_sequence_leg(now_ms);
        heartbeat_hw_uart_send_string("[balance] sequence return\r\n");
    }
    else
    {
        balance_app_set_sequence_state(BALANCE_SEQUENCE_COMPLETE);
        heartbeat_hw_uart_send_string("[balance] sequence complete\r\n");
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
                BALANCE_APP_COMMAND_ENABLE,
                emm42_set_enabled(BALANCE_APP_EMM42_ADDRESS, 0u, 0u),
                now_ms))
        {
            balance_app_set_state(BALANCE_APP_DISABLE, now_ms);
        }
    }
    else if ((BALANCE_APP_DISABLE == balance_status.state) &&
             (0u == balance_pending_command))
    {
        (void)balance_app_begin_command(
            BALANCE_APP_COMMAND_ENABLE,
            emm42_set_enabled(BALANCE_APP_EMM42_ADDRESS, 0u, 0u), now_ms);
    }
    else if ((BALANCE_APP_WAIT_LOWER_STOP == balance_status.state) &&
             ((now_ms - balance_state_start_ms) >=
              BALANCE_LOWER_STOP_SETTLE_MS) &&
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
        (void)balance_app_send_motor_target(
            0.0f, BALANCE_LEVEL_RETURN_RPM, now_ms);
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
                    balance_recovery_valid_frames = 0u;
                    balance_status.flags &=
                        (uint8)(~BALANCE_APP_FLAG_ACTIVE);
                    balance_app_set_state(BALANCE_APP_WAIT_VISION, now_ms);
                    balance_last_control_ms = now_ms;
                    balance_last_outer_control_ms = now_ms;
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
    float command_angle;
    uint8 update_output = 0u;

    if ((now_ms - balance_last_control_ms) >= BALANCE_ESTIMATOR_PERIOD_MS)
    {
        balance_last_control_ms += BALANCE_ESTIMATOR_PERIOD_MS;
        if ((now_ms - balance_last_control_ms) >= BALANCE_ESTIMATOR_PERIOD_MS)
        {
            balance_last_control_ms = now_ms;
        }
        if ((now_ms - balance_last_outer_control_ms) >=
            BALANCE_OUTER_CONTROL_PERIOD_MS)
        {
            balance_last_outer_control_ms +=
                BALANCE_OUTER_CONTROL_PERIOD_MS;
            if ((now_ms - balance_last_outer_control_ms) >=
                BALANCE_OUTER_CONTROL_PERIOD_MS)
            {
                balance_last_outer_control_ms = now_ms;
            }
            update_output = 1u;
        }
        balance_app_control_step(now_ms, update_output);
        if ((BALANCE_APP_ACTIVE == balance_status.state) &&
            (0u != balance_sequence_start_pending))
        {
            balance_sequence_start_pending = 0u;
            balance_app_begin_sequence(now_ms);
        }
        balance_app_update_sequence(now_ms);
    }

    if ((BALANCE_APP_FAULT == balance_status.state) ||
        (0u != balance_pending_command))
    {
        return;
    }
    if ((now_ms - balance_last_command_ms) >= BALANCE_COMMAND_PERIOD_MS)
    {
        command_angle = balance_status.lever_angle_deg;
        if ((0u == balance_has_sent_lever_target) ||
            (balance_app_abs(command_angle - balance_last_sent_lever_deg) >=
             BALANCE_LEVER_COMMAND_DEADBAND_DEG))
        {
            (void)balance_app_send_motor_target(
                command_angle, BALANCE_EMM42_MOVE_RPM, now_ms);
        }
    }

    if (0u != balance_motor_feedback_new)
    {
        balance_motor_feedback_new = 0u;
        follow_error = balance_app_abs(balance_status.motor_feedback_deg -
                                       balance_status.motor_target_deg);
        if (follow_error > BALANCE_MOTOR_FOLLOW_ERROR_DEG)
        {
            if (0u == balance_follow_error_active)
            {
                balance_follow_error_active = 1u;
                balance_follow_error_start_ms = now_ms;
            }
            else if ((now_ms - balance_follow_error_start_ms) >=
                     BALANCE_MOTOR_FOLLOW_ERROR_TIMEOUT_MS)
            {
                balance_status.command_error_count++;
                balance_app_enter_fault(
                    BALANCE_FAULT_MOTOR_FOLLOW_ERROR, now_ms);
            }
        }
        else
        {
            balance_follow_error_active = 0u;
        }
    }
}

static void balance_app_query_position_if_due(uint32 now_ms)
{
    uint8 query_enabled =
        ((BALANCE_APP_UNCONFIGURED == balance_status.state) ||
         ((BALANCE_APP_MOVE_LEVEL == balance_status.state) &&
          (0u != balance_level_move_acked)) ||
         (BALANCE_APP_ACTIVE == balance_status.state) ||
         (BALANCE_APP_RECOVERY == balance_status.state) ||
         (BALANCE_APP_WAIT_VISION == balance_status.state)) ? 1u : 0u;

    if ((0u == query_enabled) ||
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
    ball_motion_profile_config_t profile_config;
    balance_actuator_trajectory_config_t actuator_config;
    uint32 now_ms = heartbeat_get_ms();

    config.position_gain_s_inv = BALANCE_POSITION_LOOP_GAIN_S_INV;
    config.velocity_gain_s_inv = BALANCE_VELOCITY_LOOP_GAIN_S_INV;
    config.max_ball_velocity_mps = BALANCE_MAX_BALL_VELOCITY_MPS;
    config.rolling_factor = BALANCE_ROLLING_FACTOR;
    config.rolling_friction_accel_mps2 =
        BALANCE_ROLLING_FRICTION_ACCEL_MPS2;
    config.rail_curvature_m_inv = BALANCE_RAIL_CURVATURE_M_INV;
    config.position_correction_gain = BALANCE_ESTIMATOR_POSITION_GAIN;
    config.velocity_residual_gain =
        BALANCE_ESTIMATOR_VELOCITY_RESIDUAL_GAIN;
    config.max_ball_accel_mps2 = BALANCE_MAX_BALL_ACCEL_MPS2;
    config.brake_accel_mps2 = BALANCE_BRAKE_ACCEL_MPS2;
    config.actuator_delay_s = (float)BALANCE_ACTUATOR_DELAY_MS * 0.001f;
    config.brake_margin_delay_s =
        (float)BALANCE_BRAKE_MARGIN_DELAY_MS * 0.001f;
    config.overspeed_release_ratio = BALANCE_OVERSPEED_RELEASE_RATIO;
    config.overspeed_min_hold_ms = BALANCE_OVERSPEED_MIN_HOLD_MS;
    config.command_period_s =
        (float)BALANCE_COMMAND_PERIOD_MS * 0.001f;
    config.capture_position_m = BALANCE_CENTER_CAPTURE_POSITION_M;
    config.center_dead_position_m = BALANCE_CENTER_DEAD_POSITION_M;
    config.capture_velocity_mps = BALANCE_CAPTURE_VELOCITY_MPS;
    config.stick_velocity_mps = BALANCE_STICK_VELOCITY_MPS;
    config.capture_integral_gain = BALANCE_CAPTURE_INTEGRAL_GAIN;
    config.capture_max_accel_mps2 = BALANCE_CAPTURE_MAX_ACCEL_MPS2;
    config.breakaway_angle_deg = BALANCE_BREAKAWAY_ANGLE_DEG;
    config.breakaway_qualify_ms = BALANCE_BREAKAWAY_QUALIFY_MS;
    config.breakaway_pulse_ms = BALANCE_BREAKAWAY_PULSE_MS;
    config.breakaway_movement_m = BALANCE_BREAKAWAY_MOVEMENT_M;
    config.edge_recovery_accel_mps2 =
        BALANCE_EDGE_RECOVERY_ACCEL_MPS2;
    config.max_lever_angle_deg = BALANCE_MAX_LEVER_ANGLE_DEG;
    config.degraded_lever_angle_deg = BALANCE_DEGRADED_LEVER_ANGLE_DEG;
    config.edge_position_m = BALANCE_EDGE_POSITION_M;
    config.hard_edge_position_m = BALANCE_HARD_EDGE_POSITION_M;
    config.fresh_measurement_ms = BALANCE_FRESH_MEASUREMENT_MS;
    config.valid_measurement_ms = BALANCE_VALID_MEASUREMENT_MS;
    config.calibration_provisional = BALANCE_CALIBRATION_PROVISIONAL;
    balance_control_init(&balance_controller, &config);

    profile_config.drive_accel_mps2 = BALANCE_PROFILE_DRIVE_ACCEL_MPS2;
    profile_config.brake_accel_mps2 = BALANCE_PROFILE_BRAKE_ACCEL_MPS2;
    profile_config.max_velocity_mps = BALANCE_PROFILE_MAX_VELOCITY_MPS;
    profile_config.max_jerk_mps3 = BALANCE_PROFILE_MAX_JERK_MPS3;
    profile_config.feedforward_lead_s =
        BALANCE_PROFILE_FEEDFORWARD_LEAD_S;
    profile_config.capture_position_m =
        BALANCE_PROFILE_CAPTURE_POSITION_M;
    profile_config.capture_velocity_mps =
        BALANCE_PROFILE_CAPTURE_VELOCITY_MPS;
    profile_config.position_tolerance_m =
        BALANCE_PROFILE_POSITION_TOLERANCE_M;
    profile_config.velocity_tolerance_mps =
        BALANCE_PROFILE_VELOCITY_TOLERANCE_MPS;
    ball_motion_profile_init(&balance_profile, &profile_config);
    actuator_config.max_angle_deg = BALANCE_MAX_LEVER_ANGLE_DEG;
    actuator_config.max_rate_deg_s = BALANCE_MAX_LEVER_RATE_DEG_S;
    actuator_config.max_accel_deg_s2 = BALANCE_MAX_LEVER_ACCEL_DEG_S2;
    balance_actuator_trajectory_init(&balance_actuator_trajectory,
                                     &actuator_config);
    balance_status.state = BALANCE_APP_UNCONFIGURED;
    balance_status.fault = BALANCE_FAULT_NONE;
    balance_status.flags = 0u;
    balance_status.control_flags = 0u;
    balance_status.vision_confidence = 0u;
    balance_status.vision_raw_flags = 0u;
    balance_status.vision_raw_confidence = 0u;
    balance_status.vision_sequence = 0u;
    balance_status.vision_raw_sequence = 0u;
    balance_status.vision_age_ms = BALANCE_APP_AGE_INVALID;
    balance_status.vision_raw_position_dmm = 0;
    balance_status.vision_raw_velocity_mm_s = 0;
    balance_status.estimated_position_m = 0.0f;
    balance_status.estimated_velocity_mps = 0.0f;
    balance_status.predicted_position_m = 0.0f;
    balance_status.predicted_velocity_mps = 0.0f;
    balance_status.target_position_m = 0.0f;
    balance_status.reference_position_m = 0.0f;
    balance_status.reference_velocity_mps = 0.0f;
    balance_status.reference_accel_mps2 = 0.0f;
    balance_status.position_error_m = 0.0f;
    balance_status.velocity_command_mps = 0.0f;
    balance_status.velocity_limit_mps = 0.0f;
    balance_status.brake_distance_m = 0.0f;
    balance_status.feedforward_accel_mps2 = 0.0f;
    balance_status.feedback_accel_mps2 = 0.0f;
    balance_status.desired_ball_accel_mps2 = 0.0f;
    balance_status.car_encoder_speed_mps = 0.0f;
    balance_status.car_encoder_accel_mps2 = 0.0f;
    balance_status.car_imu_accel_mps2 = 0.0f;
    balance_status.car_feedforward_accel_mps2 = 0.0f;
    balance_status.car_sync_lever_angle_deg = 0.0f;
    balance_status.car_imu_accel_age_ms = BALANCE_APP_AGE_INVALID;
    balance_status.car_imu_accel_valid = 0u;
    balance_status.raw_lever_angle_deg = 0.0f;
    balance_status.lever_angle_deg = 0.0f;
    balance_status.actual_lever_angle_deg = 0.0f;
    balance_status.motor_target_deg = 0.0f;
    balance_status.motor_feedback_deg = 0.0f;
    balance_status.command_error_count = 0u;
    balance_status.emm42_rx_overflow_count = 0u;
    balance_status.sequence_elapsed_ms = 0u;
    balance_status.motion_phase = BALL_MOTION_PHASE_HOLD;
    balance_status.control_phase = BALANCE_CONTROL_PHASE_HOLD;
    balance_status.friction_mode = BALANCE_FRICTION_STOPPED;
    balance_status.sequence_state = BALANCE_SEQUENCE_IDLE;
    balance_rx_frame.length = 0u;
    balance_pending_command = 0u;
    balance_consecutive_command_errors = 0u;
    balance_consecutive_position_query_errors = 0u;
    balance_recovery_valid_frames = 0u;
    balance_has_accepted_measurement = 0u;
    balance_level_move_acked = 0u;
    balance_hard_edge_active = 0u;
    balance_level_tolerance_active = 0u;
    balance_motor_feedback_new = 0u;
    balance_follow_error_active = 0u;
    balance_follow_error_start_ms = 0u;
    balance_has_seen_vision_snapshot = 0u;
    balance_last_seen_vision_sequence = 0u;
    balance_last_seen_vision_boot_id = 0u;
    balance_last_sent_lever_deg = 0.0f;
    balance_has_sent_lever_target = 0u;
    balance_actuator_command_pending = 0u;
    balance_recovery_candidate_position_m = 0.0f;
    balance_recovery_candidate_valid = 0u;
    balance_sequence_start_ms = 0u;
    balance_sequence_settle_start_ms = 0u;
    balance_sequence_settle_active = 0u;
    balance_sequence_start_pending = 0u;
    balance_last_car_feedforward_debug_ms = now_ms;
    balance_state_start_ms = now_ms;
    balance_last_control_ms = now_ms;
    balance_last_outer_control_ms = now_ms;
    balance_last_command_ms = now_ms;
    balance_last_query_ms = now_ms;
    balance_last_accepted_measurement_ms = 0u;
    balance_last_measurement_latency_ms = 0u;
    balance_previous_measurement_received_ms = 0u;
    balance_hard_edge_start_ms = 0u;
    balance_level_tolerance_start_ms = 0u;

    emm42_init();
#if (BALANCE_STARTUP_CALIBRATED != 0u)
    if (0u == balance_linkage_motor_from_physical_lever_deg(
            0.0f,
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

uint8 balance_app_set_target_position_m(float target_position_m)
{
    if ((BALANCE_APP_ACTIVE != balance_status.state) ||
        (balance_app_abs(target_position_m) > BALANCE_TARGET_POSITION_LIMIT_M))
    {
        return 0u;
    }
    ball_motion_profile_set_target(&balance_profile, target_position_m);
    balance_status.sequence_elapsed_ms = 0u;
    balance_app_set_sequence_state(BALANCE_SEQUENCE_IDLE);
    return 1u;
}

static void balance_app_begin_sequence(uint32 now_ms)
{
    ball_motion_profile_reset(
        &balance_profile,
        balance_status.estimated_position_m,
        balance_status.estimated_velocity_mps);
    ball_motion_profile_set_target(
        &balance_profile, BALANCE_SEQUENCE_POSITIVE_TARGET_M);
    balance_sequence_start_ms = now_ms;
    balance_status.sequence_elapsed_ms = 0u;
    balance_app_set_sequence_state(BALANCE_SEQUENCE_TO_POSITIVE);
    heartbeat_hw_uart_send_string("[balance] sequence start\r\n");
}

uint8 balance_app_start_sequence(void)
{
    if (BALANCE_APP_ACTIVE == balance_status.state)
    {
        balance_app_begin_sequence(heartbeat_get_ms());
        return 1u;
    }
    if ((BALANCE_APP_WAIT_VISION == balance_status.state) ||
        (BALANCE_APP_RECOVERY == balance_status.state))
    {
        balance_sequence_start_pending = 1u;
        heartbeat_hw_uart_send_string("[balance] sequence queued\r\n");
        return 1u;
    }
    return 0u;
}

void balance_app_cancel_motion(void)
{
    balance_app_abort_sequence(BALANCE_SEQUENCE_IDLE);
    balance_status.sequence_elapsed_ms = 0u;
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
        if (BALANCE_APP_COMMAND_POSITION == balance_pending_command)
            balance_app_record_position_query_error(now_ms);
        else
            balance_app_record_command_error(BALANCE_FAULT_COMMAND_TIMEOUT,
                                             now_ms);
    }

    if ((BALANCE_APP_POWER_WAIT == balance_status.state) ||
        (BALANCE_APP_DISABLE == balance_status.state) ||
        (BALANCE_APP_WAIT_LOWER_STOP == balance_status.state) ||
        (BALANCE_APP_SET_REFERENCE == balance_status.state) ||
        (BALANCE_APP_ENABLE == balance_status.state) ||
        (BALANCE_APP_MOVE_LEVEL == balance_status.state))
    {
        balance_app_process_startup(now_ms);
    }
    else if ((BALANCE_APP_ACTIVE == balance_status.state) ||
             (BALANCE_APP_RECOVERY == balance_status.state) ||
             (BALANCE_APP_WAIT_VISION == balance_status.state))
    {
        balance_app_process_active(now_ms);
    }
    balance_app_query_position_if_due(now_ms);
}

const balance_app_status_t *balance_app_get_status(void)
{
    return &balance_status;
}
