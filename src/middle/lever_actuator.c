#include "lever_actuator.h"

#include <stddef.h>

#include "balance_linkage.h"
#include "emm42.h"

#define LEVER_ACK_SUCCESS                 (0x02u)
#define LEVER_CMD_ZERO                    (0x0Au)
#define LEVER_CMD_ENABLE                  (0xF3u)
#define LEVER_CMD_MOVE                    (0xFDu)
#define LEVER_CMD_POSITION                (0x36u)

static float lever_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16 lever_u32_u16(uint32 value)
{
    return (uint16)((value > 65535u) ? 65535u : value);
}

static void lever_set_state(lever_actuator_t *self,
                            lever_actuator_state_enum state,
                            uint32 now_ms)
{
    self->status.state = state;
    self->state_start_ms = now_ms;
}

static uint8 lever_motor_target(const lever_actuator_t *self,
                                float angle_deg, float *motor_deg)
{
    if (0u == balance_linkage_relative_motor_deg(
            self->config->startup_reference_angle_deg,
            (float)self->config->linkage_target_sign * angle_deg,
            motor_deg))
    {
        return 0u;
    }
    *motor_deg *= (float)self->config->motor_direction_sign;
    return 1u;
}

static void lever_latch_fault(lever_actuator_t *self,
                              lever_actuator_fault_enum fault,
                              uint32 now_ms)
{
    float motor_deg;

    if (LEVER_ACTUATOR_FAULT == self->status.state)
    {
        return;
    }
    self->status.fault = fault;
    self->pending_command = 0u;
    self->status.command_pending = 0u;
    if ((0u != lever_motor_target(self, self->config->neutral_angle_deg,
                                  &motor_deg)) &&
        (0u != emm42_move_angle(EMM42_DEFAULT_ADDRESS, motor_deg,
                                self->config->move_rpm,
                                self->config->acceleration,
                                EMM42_POSITION_ABSOLUTE, 0u)))
    {
        self->status.target_angle_deg = self->config->neutral_angle_deg;
        self->status.motor_target_deg = motor_deg;
    }
    else
    {
        (void)emm42_stop(EMM42_DEFAULT_ADDRESS, 0u);
    }
    lever_set_state(self, LEVER_ACTUATOR_FAULT, now_ms);
}

static void lever_record_error(lever_actuator_t *self,
                               lever_actuator_fault_enum fault,
                               uint32 now_ms)
{
    self->status.command_error_count++;
    if (self->consecutive_errors < 255u)
    {
        self->consecutive_errors++;
    }
    self->pending_command = 0u;
    self->status.command_pending = 0u;
    if (self->consecutive_errors >= self->config->max_consecutive_errors)
    {
        lever_latch_fault(self, fault, now_ms);
    }
}

static uint8 lever_begin_command(lever_actuator_t *self, uint8 command,
                                 uint8 sent, uint32 now_ms)
{
    if (0u == sent)
    {
        lever_record_error(self, LEVER_ACTUATOR_FAULT_COMMAND_REJECTED,
                           now_ms);
        return 0u;
    }
    /* emm42_send() discards stale hardware RX; discard its partial parser
       state at the same command boundary. */
    self->response_frame.length = 0u;
    self->pending_command = command;
    self->pending_since_ms = now_ms;
    self->status.command_pending = 1u;
    return 1u;
}

static void lever_accept_command(lever_actuator_t *self)
{
    self->pending_command = 0u;
    self->status.command_pending = 0u;
    self->consecutive_errors = 0u;
}

static void lever_handle_ack(lever_actuator_t *self, uint8 command,
                             uint8 ack, uint32 now_ms)
{
    if (LEVER_ACK_SUCCESS != ack)
    {
        lever_record_error(self, LEVER_ACTUATOR_FAULT_COMMAND_REJECTED,
                           now_ms);
        return;
    }
    lever_accept_command(self);
    if ((LEVER_ACTUATOR_DISABLING == self->status.state) &&
        (LEVER_CMD_ENABLE == command))
    {
        lever_set_state(self, LEVER_ACTUATOR_LOWER_SETTLE, now_ms);
    }
    else if ((LEVER_ACTUATOR_ZEROING == self->status.state) &&
             (LEVER_CMD_ZERO == command))
    {
        lever_set_state(self, LEVER_ACTUATOR_ENABLING, now_ms);
    }
    else if ((LEVER_ACTUATOR_ENABLING == self->status.state) &&
             (LEVER_CMD_ENABLE == command))
    {
        lever_set_state(self, LEVER_ACTUATOR_LEVELING, now_ms);
    }
    else if ((LEVER_ACTUATOR_LEVELING == self->status.state) &&
             (LEVER_CMD_MOVE == command))
    {
        self->level_move_acked = 1u;
        self->last_query_ms = now_ms - self->config->query_period_ms;
    }
}

static void lever_drain(lever_actuator_t *self, uint32 now_ms)
{
    uint8 ack;
    float position_deg;

    while (0u != emm42_read_frame(&self->response_frame))
    {
        if ((0u != self->pending_command) &&
            (0u != emm42_decode_ack(&self->response_frame,
                                    EMM42_DEFAULT_ADDRESS,
                                    self->pending_command, &ack)))
        {
            lever_handle_ack(self, self->pending_command, ack, now_ms);
        }
        else if (0u != emm42_decode_position_deg(
                     &self->response_frame, EMM42_DEFAULT_ADDRESS,
                     &position_deg))
        {
            self->status.motor_feedback_deg = position_deg;
            self->status.motor_feedback_valid = 1u;
            if (LEVER_CMD_POSITION == self->pending_command)
            {
                lever_accept_command(self);
            }
            if ((LEVER_ACTUATOR_READY == self->status.state) &&
                (lever_abs(position_deg - self->status.motor_target_deg) >
                 self->config->follow_error_deg))
            {
                if (0u == self->follow_error_active)
                {
                    self->follow_error_active = 1u;
                    self->follow_error_start_ms = now_ms;
                }
                else if ((now_ms - self->follow_error_start_ms) >=
                         self->config->follow_error_timeout_ms)
                {
                    lever_latch_fault(self,
                        LEVER_ACTUATOR_FAULT_FOLLOW_ERROR, now_ms);
                }
            }
            else
            {
                self->follow_error_active = 0u;
            }
        }
    }
}

static lever_command_result_enum lever_send_angle(lever_actuator_t *self,
                                                   float angle_deg,
                                                   uint32 now_ms,
                                                   uint8 require_ready)
{
    float motor_deg;

    if (LEVER_ACTUATOR_FAULT == self->status.state)
    {
        return LEVER_COMMAND_FAULT;
    }
    if ((0u != require_ready) &&
        (LEVER_ACTUATOR_READY != self->status.state))
    {
        return LEVER_COMMAND_NOT_READY;
    }
    if (0u != self->pending_command)
    {
        return LEVER_COMMAND_BUSY;
    }
    if (lever_abs(angle_deg) > self->config->max_abs_angle_deg)
    {
        return LEVER_COMMAND_OUT_OF_RANGE;
    }
    if ((LEVER_ACTUATOR_READY == self->status.state) &&
        (lever_abs(angle_deg - self->status.target_angle_deg) <=
         self->config->command_deadband_deg))
    {
        return LEVER_COMMAND_UNCHANGED;
    }
    if (0u == lever_motor_target(self, angle_deg, &motor_deg))
    {
        lever_latch_fault(self, LEVER_ACTUATOR_FAULT_LINKAGE, now_ms);
        return LEVER_COMMAND_FAULT;
    }
    if (0u == lever_begin_command(
            self, LEVER_CMD_MOVE,
            emm42_move_angle(EMM42_DEFAULT_ADDRESS, motor_deg,
                             self->config->move_rpm,
                             self->config->acceleration,
                             EMM42_POSITION_ABSOLUTE, 0u), now_ms))
    {
        return LEVER_COMMAND_FAULT;
    }
    self->status.target_angle_deg = angle_deg;
    self->status.motor_target_deg = motor_deg;
    return LEVER_COMMAND_STARTED;
}

void lever_actuator_init(lever_actuator_t *self,
                         const lever_actuator_config_t *config,
                         uint32 now_ms)
{
    if ((NULL == self) || (NULL == config))
    {
        return;
    }
    self->config = config;
    self->status.state = (0u != config->startup_calibrated) ?
        LEVER_ACTUATOR_POWER_WAIT : LEVER_ACTUATOR_UNCONFIGURED;
    self->status.fault = LEVER_ACTUATOR_FAULT_NONE;
    self->status.command_pending = 0u;
    self->status.motor_feedback_valid = 0u;
    self->status.target_angle_deg = config->startup_reference_angle_deg;
    self->status.motor_target_deg = 0.0f;
    self->status.motor_feedback_deg = 0.0f;
    self->status.command_error_count = 0u;
    self->status.rx_overflow_count = 0u;
    self->state_start_ms = now_ms;
    self->pending_since_ms = now_ms;
    self->last_query_ms = now_ms;
    self->response_frame.length = 0u;
    self->pending_command = 0u;
    self->consecutive_errors = 0u;
    self->level_move_acked = 0u;
    self->level_tolerance_active = 0u;
    self->follow_error_active = 0u;
    emm42_init();
}

void lever_actuator_process(lever_actuator_t *self, uint32 now_ms)
{
    lever_command_result_enum result;

    if ((NULL == self) || (NULL == self->config))
    {
        return;
    }
    lever_drain(self, now_ms);
    self->status.rx_overflow_count =
        lever_u32_u16(emm42_get_rx_overflow_count());
    if ((0u != self->pending_command) &&
        ((now_ms - self->pending_since_ms) >
         self->config->command_timeout_ms))
    {
        lever_record_error(self, LEVER_ACTUATOR_FAULT_COMMAND_TIMEOUT,
                           now_ms);
    }

    if ((LEVER_ACTUATOR_POWER_WAIT == self->status.state) &&
        ((now_ms - self->state_start_ms) >= self->config->power_wait_ms) &&
        (0u == self->pending_command))
    {
        lever_set_state(self, LEVER_ACTUATOR_DISABLING, now_ms);
        (void)lever_begin_command(self, LEVER_CMD_ENABLE,
            emm42_set_enabled(EMM42_DEFAULT_ADDRESS, 0u, 0u), now_ms);
    }
    else if ((LEVER_ACTUATOR_DISABLING == self->status.state) &&
             (0u == self->pending_command))
    {
        (void)lever_begin_command(self, LEVER_CMD_ENABLE,
            emm42_set_enabled(EMM42_DEFAULT_ADDRESS, 0u, 0u), now_ms);
    }
    else if ((LEVER_ACTUATOR_LOWER_SETTLE == self->status.state) &&
             ((now_ms - self->state_start_ms) >=
              self->config->lower_settle_ms) &&
             (0u == self->pending_command))
    {
        lever_set_state(self, LEVER_ACTUATOR_ZEROING, now_ms);
        (void)lever_begin_command(self, LEVER_CMD_ZERO,
            emm42_set_current_position_zero(EMM42_DEFAULT_ADDRESS), now_ms);
    }
    else if ((LEVER_ACTUATOR_ZEROING == self->status.state) &&
             (0u == self->pending_command))
    {
        (void)lever_begin_command(self, LEVER_CMD_ZERO,
            emm42_set_current_position_zero(EMM42_DEFAULT_ADDRESS), now_ms);
    }
    else if ((LEVER_ACTUATOR_ENABLING == self->status.state) &&
             (0u == self->pending_command))
    {
        (void)lever_begin_command(self, LEVER_CMD_ENABLE,
            emm42_set_enabled(EMM42_DEFAULT_ADDRESS, 1u, 0u), now_ms);
    }
    else if ((LEVER_ACTUATOR_LEVELING == self->status.state) &&
             (0u == self->level_move_acked) &&
             (0u == self->pending_command))
    {
        result = lever_send_angle(self, self->config->neutral_angle_deg,
                                  now_ms, 0u);
        if ((LEVER_COMMAND_STARTED != result) &&
            (LEVER_COMMAND_UNCHANGED != result))
        {
            lever_latch_fault(self, LEVER_ACTUATOR_FAULT_LEVEL_TIMEOUT,
                              now_ms);
        }
    }
    else if (LEVER_ACTUATOR_LEVELING == self->status.state)
    {
        if ((now_ms - self->state_start_ms) >=
            self->config->level_timeout_ms)
        {
            lever_latch_fault(self, LEVER_ACTUATOR_FAULT_LEVEL_TIMEOUT,
                              now_ms);
        }
        else if ((0u != self->status.motor_feedback_valid) &&
                 (lever_abs(self->status.motor_feedback_deg -
                            self->status.motor_target_deg) <=
                  self->config->level_motor_tolerance_deg))
        {
            if (0u == self->level_tolerance_active)
            {
                self->level_tolerance_active = 1u;
                self->level_tolerance_start_ms = now_ms;
            }
            else if ((now_ms - self->level_tolerance_start_ms) >=
                     self->config->level_settle_ms)
            {
                lever_set_state(self, LEVER_ACTUATOR_READY, now_ms);
            }
        }
        else
        {
            self->level_tolerance_active = 0u;
        }
    }

    if (((LEVER_ACTUATOR_LEVELING == self->status.state) ||
         (LEVER_ACTUATOR_READY == self->status.state)) &&
        (0u == self->pending_command) &&
        ((now_ms - self->last_query_ms) >= self->config->query_period_ms))
    {
        self->last_query_ms = now_ms;
        (void)lever_begin_command(self, LEVER_CMD_POSITION,
            emm42_query_position(EMM42_DEFAULT_ADDRESS), now_ms);
    }
}

lever_command_result_enum lever_actuator_command_angle(
    lever_actuator_t *self, float angle_deg, uint32 now_ms)
{
    if ((NULL == self) || (NULL == self->config))
    {
        return LEVER_COMMAND_FAULT;
    }
    return lever_send_angle(self, angle_deg, now_ms, 1u);
}

lever_command_result_enum lever_actuator_command_neutral(
    lever_actuator_t *self, uint32 now_ms)
{
    if ((NULL == self) || (NULL == self->config))
    {
        return LEVER_COMMAND_FAULT;
    }
    return lever_send_angle(self, self->config->neutral_angle_deg,
                            now_ms, 1u);
}

uint8 lever_actuator_is_ready(const lever_actuator_t *self)
{
    return ((NULL != self) &&
            (LEVER_ACTUATOR_READY == self->status.state)) ? 1u : 0u;
}

const lever_actuator_status_t *lever_actuator_get_status(
    const lever_actuator_t *self)
{
    return (NULL != self) ? &self->status : NULL;
}
