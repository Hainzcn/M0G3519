#ifndef BALL_STATE_OBSERVER_H_
#define BALL_STATE_OBSERVER_H_

#include "zf_common_typedef.h"

#define BALL_OBSERVER_UPDATE_ACCEPTED          (0x01u)
#define BALL_OBSERVER_UPDATE_SESSION_RESET     (0x02u)
#define BALL_OBSERVER_UPDATE_REJECTED          (0x04u)

typedef struct
{
    float alpha;
    float beta;
    float position_limit_m;
    float max_implied_speed_mps;
    uint32 max_capture_interval_ms;
    uint32 valid_timeout_ms;
    uint32 transport_latency_ms;
    uint32 max_prediction_ms;
    uint8 recovery_frames;
} ball_state_observer_config_t;

typedef struct
{
    float position_m;
    float velocity_mps;
    uint32 age_ms;
    uint32 capture_ms;
    uint16 sequence;
    uint16 boot_id;
    uint8 trusted_frames;
    uint8 valid;
} ball_state_observer_output_t;

typedef struct
{
    ball_state_observer_config_t config;
    ball_state_observer_output_t output;
    float last_raw_position_m;
    uint32 last_raw_capture_ms;
    uint32 last_received_ms;
    uint8 session_active;
    uint8 tracking_active;
} ball_state_observer_t;

void ball_state_observer_init(
    ball_state_observer_t *observer,
    const ball_state_observer_config_t *config);
void ball_state_observer_reset(ball_state_observer_t *observer);
uint8 ball_state_observer_update(
    ball_state_observer_t *observer,
    float measured_position_m,
    uint32 capture_ms,
    uint32 received_ms,
    uint8 processing_ms,
    uint16 sequence,
    uint16 boot_id);
void ball_state_observer_get_control_state(
    ball_state_observer_t *observer,
    uint32 now_ms,
    ball_state_observer_output_t *output);

#endif
