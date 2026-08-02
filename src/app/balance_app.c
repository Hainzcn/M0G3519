#include "balance_app.h"

#include <stddef.h>

#include "balance_config.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "lever_actuator.h"
#include "sw1_open_loop.h"
#include "v1_center_controller.h"
#include "vision_link.h"

#define BALANCE_AGE_INVALID                (0xFFFFFFFFu)

static lever_actuator_t balance_actuator;
static v1_center_controller_t balance_v1;
static sw1_open_loop_t balance_sw1;
static balance_app_status_t balance_status;
static balance_platform_motion_t balance_platform_motion;
static uint8 balance_has_measurement;
static uint8 balance_latest_measurement_acceptable;
static uint8 balance_has_seen_snapshot;
static uint16 balance_last_snapshot_sequence;
static uint16 balance_last_snapshot_boot_id;
static uint32 balance_last_measurement_received_ms;
static uint32 balance_last_measurement_latency_ms;
static uint8 balance_edge_active;
static uint32 balance_edge_start_ms;
static float balance_edge_start_abs_m;

static float balance_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 balance_measurement_acceptable(
    const vision_link_snapshot_t *measurement)
{
    uint8 required = VISION_LINK_FLAG_MEASURED_VALID |
                     VISION_LINK_FLAG_TRACKER_READY |
                     VISION_LINK_FLAG_CALIBRATION_VALID;

    return (((measurement->flags & required) == required) &&
            (measurement->confidence >=
             balance_safety_config.min_vision_confidence)) ? 1u : 0u;
}

static void balance_enter_fault(balance_app_fault_enum fault)
{
    if (BALANCE_MODE_FAULT == balance_status.mode)
    {
        return;
    }
    balance_status.mode = BALANCE_MODE_FAULT;
    balance_status.fault = fault;
    sw1_open_loop_cancel(&balance_sw1);
    (void)lever_actuator_command_neutral(&balance_actuator,
                                         heartbeat_get_ms());
    heartbeat_hw_uart_send_string("[balance] fault latched\r\n");
}

static balance_app_fault_enum balance_map_actuator_fault(
    lever_actuator_fault_enum fault)
{
    switch (fault)
    {
        case LEVER_ACTUATOR_FAULT_LINKAGE:
            return BALANCE_FAULT_LINKAGE_UNREACHABLE;
        case LEVER_ACTUATOR_FAULT_COMMAND_TIMEOUT:
            return BALANCE_FAULT_COMMAND_TIMEOUT;
        case LEVER_ACTUATOR_FAULT_COMMAND_REJECTED:
            return BALANCE_FAULT_COMMAND_REJECTED;
        case LEVER_ACTUATOR_FAULT_LEVEL_TIMEOUT:
            return BALANCE_FAULT_LEVEL_TIMEOUT;
        case LEVER_ACTUATOR_FAULT_FOLLOW_ERROR:
            return BALANCE_FAULT_MOTOR_FOLLOW_ERROR;
        default:
            return BALANCE_FAULT_NONE;
    }
}

static void balance_update_vision(uint32 now_ms,
                                  v1_center_observation_t *observation)
{
    vision_link_snapshot_t measurement;
    vision_link_status_t link_status;
    uint8 new_snapshot = 0u;
    uint32 age = BALANCE_AGE_INVALID;
    uint32 latency;

    vision_link_get_status(&link_status);
    if (0u != vision_link_get_latest_snapshot(&measurement))
    {
        new_snapshot = ((0u == balance_has_seen_snapshot) ||
            (measurement.boot_id != balance_last_snapshot_boot_id) ||
            (measurement.sequence != balance_last_snapshot_sequence)) ? 1u : 0u;
        if (0u != new_snapshot)
        {
            balance_has_seen_snapshot = 1u;
            balance_last_snapshot_boot_id = measurement.boot_id;
            balance_last_snapshot_sequence = measurement.sequence;
            balance_latest_measurement_acceptable =
                balance_measurement_acceptable(&measurement);
            if (0u != balance_latest_measurement_acceptable)
            {
                latency = (uint32)measurement.processing_ms +
                    balance_safety_config.vision_transport_latency_ms;
                if (latency > balance_safety_config.vision_max_compensation_ms)
                {
                    latency = balance_safety_config.vision_max_compensation_ms;
                }
                balance_has_measurement = 1u;
                balance_last_measurement_received_ms = measurement.received_ms;
                balance_last_measurement_latency_ms = latency;
                balance_status.vision_sequence = measurement.sequence;
                balance_status.vision_confidence = measurement.confidence;
                balance_status.position_m =
                    (float)measurement.position_dmm * 0.0001f +
                    (float)measurement.velocity_mm_s * 0.001f *
                    (float)latency * 0.001f;
                balance_status.velocity_mps =
                    (float)measurement.velocity_mm_s * 0.001f;
            }
        }
    }
    if (0u != balance_has_measurement)
    {
        age = now_ms - balance_last_measurement_received_ms;
        if (age <= BALANCE_AGE_INVALID - balance_last_measurement_latency_ms)
        {
            age += balance_last_measurement_latency_ms;
        }
        else
        {
            age = BALANCE_AGE_INVALID;
        }
    }
    balance_status.vision_age_ms = age;
    observation->valid = ((0u != link_status.link_online) &&
        (0u != balance_has_measurement) &&
        (0u != balance_latest_measurement_acceptable) &&
        (age <= balance_v1_config.max_measurement_age_ms)) ? 1u : 0u;
    observation->new_measurement =
        ((0u != new_snapshot) && (0u != observation->valid)) ? 1u : 0u;
    observation->position_m = balance_status.position_m;
    observation->velocity_mps = balance_status.velocity_mps;
    observation->age_ms = age;

    if (0u != link_status.link_online)
    {
        balance_status.flags |= BALANCE_APP_FLAG_VISION_ONLINE;
    }
    else
    {
        balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_VISION_ONLINE);
    }
}

static uint8 balance_apply_angle(float angle_deg, uint32 now_ms)
{
    lever_command_result_enum result = lever_actuator_command_angle(
        &balance_actuator, angle_deg, now_ms);

    if ((LEVER_COMMAND_STARTED == result) ||
        (LEVER_COMMAND_UNCHANGED == result))
    {
        return 1u;
    }
    if ((LEVER_COMMAND_BUSY == result) ||
        (LEVER_COMMAND_NOT_READY == result))
    {
        return 0u;
    }
    balance_enter_fault((LEVER_COMMAND_OUT_OF_RANGE == result) ?
        BALANCE_FAULT_LINKAGE_UNREACHABLE : BALANCE_FAULT_COMMAND_REJECTED);
    return 0u;
}

static void balance_process_edge(const v1_center_observation_t *observation,
                                 uint32 now_ms)
{
    float position_abs;

    if (0u == observation->valid)
    {
        return;
    }
    position_abs = balance_abs(observation->position_m);
    if (position_abs >= balance_safety_config.soft_edge_position_m)
    {
        balance_status.flags |= BALANCE_APP_FLAG_SOFT_EDGE;
    }
    else
    {
        balance_status.flags &= (uint8)(~BALANCE_APP_FLAG_SOFT_EDGE);
    }
    if (position_abs >= balance_safety_config.hard_edge_position_m)
    {
        if (0u == balance_edge_active)
        {
            balance_edge_active = 1u;
            balance_edge_start_ms = now_ms;
            balance_edge_start_abs_m = position_abs;
            sw1_open_loop_cancel(&balance_sw1);
            balance_status.mode = BALANCE_MODE_EDGE_RECOVERY;
            v1_center_controller_begin(&balance_v1,
                observation->position_m, observation->velocity_mps, now_ms);
        }
        else if (((now_ms - balance_edge_start_ms) >=
                  balance_safety_config.edge_progress_timeout_ms) &&
                 ((balance_edge_start_abs_m - position_abs) <
                  balance_safety_config.edge_progress_m))
        {
            balance_enter_fault(BALANCE_FAULT_EDGE_NO_PROGRESS);
        }
    }
    else if ((0u != balance_edge_active) &&
             (position_abs < balance_safety_config.soft_edge_position_m))
    {
        balance_edge_active = 0u;
        if (BALANCE_MODE_EDGE_RECOVERY == balance_status.mode)
        {
            balance_status.mode = BALANCE_MODE_V1;
        }
    }
}

static void balance_process_v1(const v1_center_observation_t *observation,
                               uint32 now_ms)
{
    const v1_center_output_t *output;

    v1_center_controller_step(&balance_v1, observation, now_ms);
    output = v1_center_controller_get_output(&balance_v1);
    balance_status.phase = (uint8)output->phase;
    balance_status.remaining_m = output->remaining_m;
    balance_status.brake_distance_m = output->brake_distance_m;
    (void)balance_apply_angle(output->target_angle_deg, now_ms);
    if (V1_CENTER_FAULT_CAPTURE_TIMEOUT == output->fault)
    {
        balance_enter_fault(BALANCE_FAULT_V1_CAPTURE_TIMEOUT);
    }
}

static void balance_process_sw1(uint32 now_ms)
{
    const sw1_open_loop_output_t *output;

    sw1_open_loop_step(&balance_sw1, now_ms);
    output = sw1_open_loop_get_output(&balance_sw1);
    balance_status.phase = (uint8)output->phase;
    balance_status.sw1_elapsed_ms = output->elapsed_ms;
    if (SW1_OPEN_LOOP_FAULT_DEADLINE_MISSED == output->fault)
    {
        balance_enter_fault(BALANCE_FAULT_SW1_DEADLINE_MISSED);
        return;
    }
    if (SW1_OPEN_LOOP_FAULT_TOTAL_TIMEOUT == output->fault)
    {
        balance_enter_fault(BALANCE_FAULT_SW1_TIMEOUT);
        return;
    }
    if (0u != output->command_due)
    {
        if (0u != balance_apply_angle(output->target_angle_deg, now_ms))
        {
            sw1_open_loop_mark_command_applied(&balance_sw1);
        }
    }
    if (SW1_OPEN_LOOP_COMPLETE == output->phase)
    {
        balance_status.mode = BALANCE_MODE_COMPLETE;
    }
}

static void balance_refresh_status(void)
{
    const lever_actuator_status_t *actuator =
        lever_actuator_get_status(&balance_actuator);
    const sw1_open_loop_output_t *sw1 = sw1_open_loop_get_output(&balance_sw1);

    balance_status.lever_target_deg = actuator->target_angle_deg;
    balance_status.motor_target_deg = actuator->motor_target_deg;
    balance_status.motor_feedback_deg = actuator->motor_feedback_deg;
    balance_status.command_error_count = actuator->command_error_count;
    balance_status.emm42_rx_overflow_count = actuator->rx_overflow_count;
    balance_status.flags &= (BALANCE_APP_FLAG_VISION_ONLINE |
                             BALANCE_APP_FLAG_MEASUREMENT_FRESH |
                             BALANCE_APP_FLAG_SOFT_EDGE);
    if (0u != lever_actuator_is_ready(&balance_actuator))
    {
        balance_status.flags |= BALANCE_APP_FLAG_ACTUATOR_READY;
    }
    if (0u != actuator->motor_feedback_valid)
    {
        balance_status.flags |= BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID;
    }
    if (0u != actuator->command_pending)
    {
        balance_status.flags |= BALANCE_APP_FLAG_COMMAND_PENDING;
    }
    if (BALANCE_MODE_FAULT == balance_status.mode)
    {
        balance_status.flags |= BALANCE_APP_FLAG_FAULT_LATCHED;
    }
    if (0u != sw1->active)
    {
        balance_status.flags |= BALANCE_APP_FLAG_SW1_ACTIVE;
    }
}

void balance_app_init(void)
{
    uint32 now_ms = heartbeat_get_ms();

    lever_actuator_init(&balance_actuator, &balance_lever_config, now_ms);
    v1_center_controller_init(&balance_v1, &balance_v1_config);
    sw1_open_loop_init(&balance_sw1, &balance_sw1_config);
    balance_status.mode = BALANCE_MODE_STARTUP;
    balance_status.phase = 0u;
    balance_status.fault = BALANCE_FAULT_NONE;
    balance_status.flags = 0u;
    balance_status.vision_sequence = 0u;
    balance_status.vision_age_ms = BALANCE_AGE_INVALID;
    balance_status.vision_confidence = 0u;
    balance_status.position_m = 0.0f;
    balance_status.velocity_mps = 0.0f;
    balance_status.remaining_m = 0.0f;
    balance_status.brake_distance_m = 0.0f;
    balance_status.sw1_elapsed_ms = BALANCE_AGE_INVALID;
    balance_has_measurement = 0u;
    balance_latest_measurement_acceptable = 0u;
    balance_has_seen_snapshot = 0u;
    balance_edge_active = 0u;
    balance_platform_motion.valid = 0u;
}

void balance_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();
    v1_center_observation_t observation;
    const lever_actuator_status_t *actuator;

    lever_actuator_process(&balance_actuator, now_ms);
    actuator = lever_actuator_get_status(&balance_actuator);
    balance_update_vision(now_ms, &observation);
    if (0u != observation.valid)
    {
        balance_status.flags |= BALANCE_APP_FLAG_MEASUREMENT_FRESH;
    }
    else
    {
        balance_status.flags &=
            (uint8)(~BALANCE_APP_FLAG_MEASUREMENT_FRESH);
    }

    if ((LEVER_ACTUATOR_FAULT == actuator->state) &&
        (BALANCE_MODE_FAULT != balance_status.mode))
    {
        balance_enter_fault(balance_map_actuator_fault(actuator->fault));
    }
    if ((BALANCE_MODE_STARTUP == balance_status.mode) &&
        (0u != lever_actuator_is_ready(&balance_actuator)))
    {
        balance_status.mode = BALANCE_MODE_V1;
        v1_center_controller_reset(&balance_v1);
        heartbeat_hw_uart_send_string("[balance] actuator ready\r\n");
    }
    balance_process_edge(&observation, now_ms);

    if ((BALANCE_MODE_V1 == balance_status.mode) ||
        (BALANCE_MODE_EDGE_RECOVERY == balance_status.mode))
    {
        balance_process_v1(&observation, now_ms);
    }
    else if (BALANCE_MODE_SW1 == balance_status.mode)
    {
        balance_process_sw1(now_ms);
    }
    else if (BALANCE_MODE_COMPLETE == balance_status.mode)
    {
        balance_status.phase = (uint8)SW1_OPEN_LOOP_COMPLETE;
        (void)lever_actuator_command_neutral(&balance_actuator, now_ms);
    }
    else if (BALANCE_MODE_FAULT == balance_status.mode)
    {
        (void)lever_actuator_command_neutral(&balance_actuator, now_ms);
    }
    else
    {
        balance_status.phase = (uint8)actuator->state;
    }
    balance_refresh_status();
}

balance_request_result_t balance_app_start_sw1(void)
{
    uint32 now_ms = heartbeat_get_ms();
    const lever_actuator_status_t *actuator =
        lever_actuator_get_status(&balance_actuator);
    lever_command_result_enum result;

    if ((BALANCE_MODE_FAULT == balance_status.mode) ||
        (LEVER_ACTUATOR_FAULT == actuator->state))
    {
        return BALANCE_REQUEST_FAULT;
    }
    if (0u == lever_actuator_is_ready(&balance_actuator))
    {
        return BALANCE_REQUEST_NOT_READY;
    }
    if ((0u != actuator->command_pending) ||
        (0u != sw1_open_loop_get_output(&balance_sw1)->active))
    {
        return BALANCE_REQUEST_BUSY;
    }
    result = lever_actuator_command_angle(&balance_actuator,
        sw1_open_loop_get_start_angle(&balance_sw1), now_ms);
    if ((LEVER_COMMAND_STARTED != result) &&
        (LEVER_COMMAND_UNCHANGED != result))
    {
        return (LEVER_COMMAND_BUSY == result) ?
            BALANCE_REQUEST_BUSY : BALANCE_REQUEST_FAULT;
    }
    v1_center_controller_reset(&balance_v1);
    sw1_open_loop_start(&balance_sw1, now_ms);
    balance_status.mode = BALANCE_MODE_SW1;
    balance_status.sw1_elapsed_ms = 0u;
    balance_status.remaining_m = 0.0f;
    balance_status.brake_distance_m = 0.0f;
    heartbeat_hw_uart_send_string("[balance] SW1 start\r\n");
    return BALANCE_REQUEST_ACCEPTED;
}

void balance_app_cancel(void)
{
    if (BALANCE_MODE_FAULT == balance_status.mode)
    {
        return;
    }
    sw1_open_loop_cancel(&balance_sw1);
    v1_center_controller_reset(&balance_v1);
    balance_status.mode = BALANCE_MODE_V1;
    balance_status.sw1_elapsed_ms = BALANCE_AGE_INVALID;
    (void)lever_actuator_command_neutral(&balance_actuator,
                                         heartbeat_get_ms());
}

const balance_app_status_t *balance_app_get_status(void)
{
    return &balance_status;
}

void balance_app_set_platform_motion(const balance_platform_motion_t *motion)
{
    if (NULL != motion)
    {
        balance_platform_motion = *motion;
    }
}
