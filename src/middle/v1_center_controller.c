#include "v1_center_controller.h"

#include <stddef.h>

static float v1_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static const v1_center_direction_config_t *v1_direction_config(
    const v1_center_controller_t *self)
{
    return (self->output.direction > 0) ?
        &self->config->positive : &self->config->negative;
}

static void v1_set_phase(v1_center_controller_t *self,
                         v1_center_phase_enum phase, uint32 now_ms)
{
    self->output.phase = phase;
    self->phase_start_ms = now_ms;
    self->settle_valid = 0u;
}

static void v1_update_target(v1_center_controller_t *self)
{
    const v1_center_direction_config_t *direction;

    direction = v1_direction_config(self);
    if (V1_CENTER_ACCEL == self->output.phase)
    {
        self->output.target_angle_deg = self->config->neutral_angle_deg +
            direction->drive_angle_offset_deg;
    }
    else if (V1_CENTER_BRAKE == self->output.phase)
    {
        self->output.target_angle_deg = self->config->neutral_angle_deg +
            direction->brake_angle_offset_deg;
    }
    else
    {
        self->output.target_angle_deg = self->config->neutral_angle_deg;
    }
}

static void v1_update_motion_metrics(v1_center_controller_t *self,
                                     float position_m, float velocity_mps,
                                     float *toward_velocity_mps)
{
    const v1_center_direction_config_t *direction;
    float toward_velocity;
    float speed;
    float remaining;

    direction = v1_direction_config(self);
    toward_velocity = (float)self->output.direction * velocity_mps;
    speed = (toward_velocity > 0.0f) ? toward_velocity : 0.0f;
    remaining = (float)self->output.direction * -position_m;
    if (remaining < 0.0f)
    {
        remaining = 0.0f;
    }
    self->output.remaining_m = remaining;
    self->output.brake_distance_m = self->config->brake_margin_m +
        speed * self->config->brake_delay_s +
        speed * speed / (2.0f * direction->brake_decel_mps2);
    *toward_velocity_mps = toward_velocity;
}

void v1_center_controller_begin(v1_center_controller_t *self,
                                float position_m, float velocity_mps,
                                uint32 now_ms)
{
    if ((NULL == self) || (NULL == self->config))
    {
        return;
    }
    if (position_m < 0.0f)
    {
        self->output.direction = 1;
    }
    else if (position_m > 0.0f)
    {
        self->output.direction = -1;
    }
    else
    {
        self->output.direction = (velocity_mps > 0.0f) ? -1 : 1;
    }
    if (0u == self->capture_active)
    {
        self->capture_active = 1u;
        self->capture_start_ms = now_ms;
    }
    v1_set_phase(self, V1_CENTER_ACCEL, now_ms);
    v1_update_target(self);
}

void v1_center_controller_init(v1_center_controller_t *self,
                               const v1_center_config_t *config)
{
    if (NULL == self)
    {
        return;
    }
    self->config = config;
    v1_center_controller_reset(self);
}

void v1_center_controller_reset(v1_center_controller_t *self)
{
    if (NULL == self)
    {
        return;
    }
    self->output.phase = V1_CENTER_WAIT_VISION;
    self->output.fault = V1_CENTER_FAULT_NONE;
    self->output.direction = 1;
    self->output.target_angle_deg =
        (NULL != self->config) ? self->config->neutral_angle_deg : 0.0f;
    self->output.remaining_m = 0.0f;
    self->output.brake_distance_m = 0.0f;
    self->phase_start_ms = 0u;
    self->capture_start_ms = 0u;
    self->settle_valid_start_ms = 0u;
    self->capture_active = 0u;
    self->settle_valid = 0u;
    self->recovery_valid_count = 0u;
}

void v1_center_controller_step(v1_center_controller_t *self,
                               const v1_center_observation_t *observation,
                               uint32 now_ms)
{
    float position_abs;
    float velocity_abs;
    float toward_velocity;

    if ((NULL == self) || (NULL == self->config) ||
        (NULL == observation) ||
        (V1_CENTER_FAULT_NONE != self->output.fault))
    {
        return;
    }
    if ((0u == observation->valid) ||
        (observation->age_ms > self->config->max_measurement_age_ms))
    {
        self->output.phase = V1_CENTER_WAIT_VISION;
        self->output.target_angle_deg = self->config->neutral_angle_deg;
        self->output.remaining_m = 0.0f;
        self->output.brake_distance_m = 0.0f;
        self->capture_active = 0u;
        self->settle_valid = 0u;
        self->recovery_valid_count = 0u;
        return;
    }
    if ((0u != self->capture_active) &&
        ((now_ms - self->capture_start_ms) >=
         self->config->capture_timeout_ms))
    {
        self->output.fault = V1_CENTER_FAULT_CAPTURE_TIMEOUT;
        self->output.target_angle_deg = self->config->neutral_angle_deg;
        return;
    }
    if (0u == observation->new_measurement)
    {
        return;
    }

    position_abs = v1_abs(observation->position_m);
    velocity_abs = v1_abs(observation->velocity_mps);
    if (V1_CENTER_WAIT_VISION == self->output.phase)
    {
        if (self->recovery_valid_count < 255u)
        {
            self->recovery_valid_count++;
        }
        if (self->recovery_valid_count < self->config->recovery_valid_frames)
        {
            return;
        }
        self->recovery_valid_count = 0u;
        if ((position_abs <= self->config->hold_enter_position_m) &&
            (velocity_abs <= self->config->stop_velocity_mps))
        {
            v1_set_phase(self, V1_CENTER_SETTLE, now_ms);
        }
        else
        {
            v1_center_controller_begin(self, observation->position_m,
                                       observation->velocity_mps, now_ms);
        }
        v1_update_target(self);
        return;
    }

    if (V1_CENTER_HOLD == self->output.phase)
    {
        if ((position_abs >= self->config->hold_exit_position_m) ||
            (velocity_abs >= self->config->hold_exit_velocity_mps))
        {
            v1_center_controller_begin(self, observation->position_m,
                                       observation->velocity_mps, now_ms);
        }
        v1_update_target(self);
        return;
    }

    if (V1_CENTER_SETTLE == self->output.phase)
    {
        if ((position_abs <= self->config->hold_enter_position_m) &&
            (velocity_abs <= self->config->stop_velocity_mps))
        {
            if (0u == self->settle_valid)
            {
                self->settle_valid = 1u;
                self->settle_valid_start_ms = now_ms;
            }
            else if ((now_ms - self->settle_valid_start_ms) >=
                     self->config->settle_ms)
            {
                v1_set_phase(self, V1_CENTER_HOLD, now_ms);
                self->capture_active = 0u;
            }
        }
        else
        {
            self->settle_valid = 0u;
            if ((now_ms - self->phase_start_ms) >= self->config->settle_ms)
            {
                v1_center_controller_begin(self, observation->position_m,
                                           observation->velocity_mps,
                                           now_ms);
            }
        }
        v1_update_target(self);
        return;
    }

    v1_update_motion_metrics(self, observation->position_m,
                             observation->velocity_mps, &toward_velocity);
    if (V1_CENTER_BRAKE == self->output.phase)
    {
        if (toward_velocity <= self->config->stop_velocity_mps)
        {
            v1_set_phase(self, V1_CENTER_SETTLE, now_ms);
        }
    }
    else if ((toward_velocity > 0.0f) &&
             (self->output.remaining_m <=
              self->output.brake_distance_m))
    {
        v1_set_phase(self, V1_CENTER_BRAKE, now_ms);
    }
    else if ((V1_CENTER_ACCEL == self->output.phase) &&
             (toward_velocity >= self->config->max_velocity_mps))
    {
        v1_set_phase(self, V1_CENTER_CRUISE, now_ms);
    }
    else if ((V1_CENTER_CRUISE == self->output.phase) &&
             (toward_velocity < self->config->resume_velocity_mps))
    {
        v1_set_phase(self, V1_CENTER_ACCEL, now_ms);
    }
    v1_update_target(self);
}

const v1_center_output_t *v1_center_controller_get_output(
    const v1_center_controller_t *self)
{
    return (NULL != self) ? &self->output : NULL;
}
