#include "sw1_open_loop.h"

#include <stddef.h>

static uint32 sw1_phase_deadline(const sw1_open_loop_t *self,
                                 sw1_open_loop_phase_enum phase)
{
    uint32 deadline = self->start_ms;

    if (phase > SW1_OPEN_LOOP_POS_ACCEL)
    {
        deadline += self->config->pulse[0].duration_ms;
    }
    if (phase > SW1_OPEN_LOOP_POS_BRAKE)
    {
        deadline += self->config->pulse[1].duration_ms;
    }
    if (phase > SW1_OPEN_LOOP_TURN_DWELL)
    {
        deadline += self->config->turn_dwell_ms;
    }
    if (phase > SW1_OPEN_LOOP_NEG_ACCEL)
    {
        deadline += self->config->pulse[2].duration_ms;
    }
    if (phase > SW1_OPEN_LOOP_NEG_BRAKE)
    {
        deadline += self->config->pulse[3].duration_ms;
    }
    if (phase > SW1_OPEN_LOOP_FINAL_SETTLE)
    {
        deadline += self->config->final_settle_ms;
    }
    return deadline;
}

static void sw1_set_phase(sw1_open_loop_t *self,
                          sw1_open_loop_phase_enum phase)
{
    self->output.phase = phase;
    self->output.command_due = 1u;
    self->command_deadline_ms = sw1_phase_deadline(self, phase);
    switch (phase)
    {
        case SW1_OPEN_LOOP_POS_ACCEL:
            self->output.target_angle_deg = self->config->neutral_angle_deg +
                self->config->pulse[0].angle_offset_deg;
            break;
        case SW1_OPEN_LOOP_POS_BRAKE:
            self->output.target_angle_deg = self->config->neutral_angle_deg +
                self->config->pulse[1].angle_offset_deg;
            break;
        case SW1_OPEN_LOOP_NEG_ACCEL:
            self->output.target_angle_deg = self->config->neutral_angle_deg +
                self->config->pulse[2].angle_offset_deg;
            break;
        case SW1_OPEN_LOOP_NEG_BRAKE:
            self->output.target_angle_deg = self->config->neutral_angle_deg +
                self->config->pulse[3].angle_offset_deg;
            break;
        default:
            self->output.target_angle_deg = self->config->neutral_angle_deg;
            break;
    }
}

void sw1_open_loop_init(sw1_open_loop_t *self,
                        const sw1_open_loop_config_t *config)
{
    if (NULL == self)
    {
        return;
    }
    self->config = config;
    self->output.phase = SW1_OPEN_LOOP_IDLE;
    self->output.fault = SW1_OPEN_LOOP_FAULT_NONE;
    self->output.target_angle_deg =
        (NULL != config) ? config->neutral_angle_deg : 0.0f;
    self->output.elapsed_ms = 0u;
    self->output.active = 0u;
    self->output.command_due = 0u;
    self->start_ms = 0u;
    self->command_deadline_ms = 0u;
}

float sw1_open_loop_get_start_angle(const sw1_open_loop_t *self)
{
    if ((NULL == self) || (NULL == self->config))
    {
        return 0.0f;
    }
    return self->config->neutral_angle_deg +
        self->config->pulse[0].angle_offset_deg;
}

void sw1_open_loop_start(sw1_open_loop_t *self, uint32 now_ms)
{
    if ((NULL == self) || (NULL == self->config))
    {
        return;
    }
    self->start_ms = now_ms;
    self->output.elapsed_ms = 0u;
    self->output.fault = SW1_OPEN_LOOP_FAULT_NONE;
    self->output.active = 1u;
    sw1_set_phase(self, SW1_OPEN_LOOP_POS_ACCEL);
    self->output.command_due = 0u;
}

void sw1_open_loop_step(sw1_open_loop_t *self, uint32 now_ms)
{
    uint32 phase_end_ms;

    if ((NULL == self) || (NULL == self->config) ||
        (0u == self->output.active))
    {
        return;
    }
    self->output.elapsed_ms = now_ms - self->start_ms;
    if (self->output.elapsed_ms >= self->config->total_timeout_ms)
    {
        self->output.fault = SW1_OPEN_LOOP_FAULT_TOTAL_TIMEOUT;
        self->output.active = 0u;
        sw1_set_phase(self, SW1_OPEN_LOOP_FAULT);
        return;
    }
    if (0u != self->output.command_due)
    {
        if ((now_ms - self->command_deadline_ms) >
            self->config->max_deadline_late_ms)
        {
            self->output.fault = SW1_OPEN_LOOP_FAULT_DEADLINE_MISSED;
            self->output.active = 0u;
            sw1_set_phase(self, SW1_OPEN_LOOP_FAULT);
        }
        return;
    }

    phase_end_ms = sw1_phase_deadline(
        self, (sw1_open_loop_phase_enum)(self->output.phase + 1));
    if (now_ms < phase_end_ms)
    {
        return;
    }
    if ((now_ms - phase_end_ms) > self->config->max_deadline_late_ms)
    {
        self->output.fault = SW1_OPEN_LOOP_FAULT_DEADLINE_MISSED;
        self->output.active = 0u;
        sw1_set_phase(self, SW1_OPEN_LOOP_FAULT);
        return;
    }

    if (SW1_OPEN_LOOP_FINAL_SETTLE == self->output.phase)
    {
        self->output.phase = SW1_OPEN_LOOP_COMPLETE;
        self->output.active = 0u;
        self->output.command_due = 0u;
        self->output.target_angle_deg = self->config->neutral_angle_deg;
        return;
    }
    sw1_set_phase(self,
        (sw1_open_loop_phase_enum)(self->output.phase + 1));
}

void sw1_open_loop_mark_command_applied(sw1_open_loop_t *self)
{
    if (NULL != self)
    {
        self->output.command_due = 0u;
    }
}

void sw1_open_loop_cancel(sw1_open_loop_t *self)
{
    if ((NULL == self) || (NULL == self->config))
    {
        return;
    }
    self->output.phase = SW1_OPEN_LOOP_CANCELED;
    self->output.fault = SW1_OPEN_LOOP_FAULT_NONE;
    self->output.target_angle_deg = self->config->neutral_angle_deg;
    self->output.active = 0u;
    self->output.command_due = 0u;
}

const sw1_open_loop_output_t *sw1_open_loop_get_output(
    const sw1_open_loop_t *self)
{
    return (NULL != self) ? &self->output : NULL;
}
