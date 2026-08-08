#include "ball_state_observer.h"

#include <string.h>

static float observer_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 observer_position_valid(
    const ball_state_observer_t *observer, float position_m)
{
    return ((position_m >= -observer->config.position_limit_m) &&
            (position_m <= observer->config.position_limit_m)) ? 1u : 0u;
}

static void observer_start_track(
    ball_state_observer_t *observer,
    float position_m,
    uint32 capture_ms,
    uint32 received_ms,
    uint8 processing_ms,
    uint16 sequence,
    uint16 boot_id)
{
    observer->output.position_m = position_m;
    observer->output.velocity_mps = 0.0f;
    observer->output.capture_ms = capture_ms;
    observer->output.sequence = sequence;
    observer->output.boot_id = boot_id;
    observer->output.age_ms =
        (uint32)processing_ms + observer->config.transport_latency_ms;
    observer->output.trusted_frames = 1u;
    observer->output.valid =
        (observer->config.recovery_frames <= 1u) ? 1u : 0u;
    observer->last_raw_position_m = position_m;
    observer->last_raw_capture_ms = capture_ms;
    observer->last_received_ms = received_ms;
    observer->session_active = 1u;
    observer->tracking_active = 1u;
}

void ball_state_observer_init(
    ball_state_observer_t *observer,
    const ball_state_observer_config_t *config)
{
    if ((NULL == observer) || (NULL == config))
    {
        return;
    }
    memset(observer, 0, sizeof(*observer));
    observer->config = *config;
}

void ball_state_observer_reset(ball_state_observer_t *observer)
{
    ball_state_observer_config_t config;

    if (NULL == observer)
    {
        return;
    }
    config = observer->config;
    memset(observer, 0, sizeof(*observer));
    observer->config = config;
}

uint8 ball_state_observer_update(
    ball_state_observer_t *observer,
    float measured_position_m,
    uint32 capture_ms,
    uint32 received_ms,
    uint8 processing_ms,
    uint16 sequence,
    uint16 boot_id)
{
    uint32 capture_delta_ms;
    float dt_s;
    float implied_speed_mps;
    float predicted_position_m;
    float residual_m;

    if ((NULL == observer) ||
        (0u == observer_position_valid(observer, measured_position_m)))
    {
        if (NULL != observer)
        {
            observer->output.valid = 0u;
            observer->output.trusted_frames = 0u;
        }
        return BALL_OBSERVER_UPDATE_REJECTED;
    }

    if ((0u == observer->session_active) ||
        (boot_id != observer->output.boot_id))
    {
        observer_start_track(observer, measured_position_m, capture_ms,
                             received_ms, processing_ms, sequence, boot_id);
        return BALL_OBSERVER_UPDATE_ACCEPTED |
               BALL_OBSERVER_UPDATE_SESSION_RESET;
    }

    capture_delta_ms = capture_ms - observer->last_raw_capture_ms;
    if (0u == capture_delta_ms)
    {
        return BALL_OBSERVER_UPDATE_REJECTED;
    }
    if ((capture_delta_ms >= 0x80000000u) ||
        (capture_delta_ms > observer->config.max_capture_interval_ms))
    {
        observer_start_track(observer, measured_position_m, capture_ms,
                             received_ms, processing_ms, sequence, boot_id);
        return BALL_OBSERVER_UPDATE_ACCEPTED |
               BALL_OBSERVER_UPDATE_SESSION_RESET;
    }

    dt_s = (float)capture_delta_ms * 0.001f;
    implied_speed_mps = observer_abs(
        measured_position_m - observer->last_raw_position_m) / dt_s;
    if (implied_speed_mps > observer->config.max_implied_speed_mps)
    {
        observer->last_raw_position_m = measured_position_m;
        observer->last_raw_capture_ms = capture_ms;
        observer->tracking_active = 0u;
        observer->output.trusted_frames = 0u;
        observer->output.valid = 0u;
        return BALL_OBSERVER_UPDATE_REJECTED;
    }

    if (0u == observer->tracking_active)
    {
        observer_start_track(observer, measured_position_m, capture_ms,
                             received_ms, processing_ms, sequence, boot_id);
        return BALL_OBSERVER_UPDATE_ACCEPTED |
               BALL_OBSERVER_UPDATE_SESSION_RESET;
    }

    predicted_position_m = observer->output.position_m +
                           observer->output.velocity_mps * dt_s;
    residual_m = measured_position_m - predicted_position_m;
    observer->output.position_m = predicted_position_m +
        observer->config.alpha * residual_m;
    if (observer->output.position_m > observer->config.position_limit_m)
        observer->output.position_m = observer->config.position_limit_m;
    else if (observer->output.position_m < -observer->config.position_limit_m)
        observer->output.position_m = -observer->config.position_limit_m;
    observer->output.velocity_mps +=
        observer->config.beta * residual_m / dt_s;
    observer->output.capture_ms = capture_ms;
    observer->output.sequence = sequence;
    observer->last_raw_position_m = measured_position_m;
    observer->last_raw_capture_ms = capture_ms;
    observer->last_received_ms = received_ms;
    if (observer->output.trusted_frames < observer->config.recovery_frames)
    {
        observer->output.trusted_frames++;
    }
    observer->output.valid =
        (observer->output.trusted_frames >=
         observer->config.recovery_frames) ? 1u : 0u;
    observer->output.age_ms =
        (uint32)processing_ms + observer->config.transport_latency_ms;
    return BALL_OBSERVER_UPDATE_ACCEPTED;
}

void ball_state_observer_get_control_state(
    ball_state_observer_t *observer,
    uint32 now_ms,
    ball_state_observer_output_t *output)
{
    uint32 age_ms;
    uint32 prediction_ms;

    if ((NULL == observer) || (NULL == output))
    {
        return;
    }
    *output = observer->output;
    if (0u == observer->tracking_active)
    {
        output->age_ms = 0xFFFFFFFFu;
        output->valid = 0u;
        return;
    }
    age_ms = observer->output.age_ms +
             (now_ms - observer->last_received_ms);
    output->age_ms = age_ms;
    if (age_ms > observer->config.valid_timeout_ms)
    {
        observer->output.valid = 0u;
        observer->output.trusted_frames = 0u;
        observer->tracking_active = 0u;
        output->valid = 0u;
        return;
    }
    prediction_ms = (age_ms > observer->config.max_prediction_ms) ?
        observer->config.max_prediction_ms : age_ms;
    output->position_m = observer->output.position_m +
        observer->output.velocity_mps * ((float)prediction_ms * 0.001f);
    if (output->position_m > observer->config.position_limit_m)
        output->position_m = observer->config.position_limit_m;
    else if (output->position_m < -observer->config.position_limit_m)
        output->position_m = -observer->config.position_limit_m;
}
