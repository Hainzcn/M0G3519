#ifndef BALL_MOTION_PROFILE_H_
#define BALL_MOTION_PROFILE_H_

#include "zf_common_typedef.h"

typedef enum
{
    BALL_MOTION_PHASE_HOLD = 0,
    BALL_MOTION_PHASE_ACCEL,
    BALL_MOTION_PHASE_CRUISE,
    BALL_MOTION_PHASE_BRAKE,
} ball_motion_phase_enum;

typedef struct
{
    float drive_accel_mps2;
    float brake_accel_mps2;
    float max_velocity_mps;
    float position_tolerance_m;
    float velocity_tolerance_mps;
} ball_motion_profile_config_t;

typedef struct
{
    float target_position_m;
    float position_m;
    float velocity_mps;
    float accel_mps2;
    ball_motion_phase_enum phase;
} ball_motion_profile_output_t;

typedef struct
{
    ball_motion_profile_config_t config;
    ball_motion_profile_output_t output;
} ball_motion_profile_t;

void ball_motion_profile_init(
    ball_motion_profile_t *profile,
    const ball_motion_profile_config_t *config);
void ball_motion_profile_reset(
    ball_motion_profile_t *profile,
    float position_m,
    float velocity_mps);
void ball_motion_profile_set_target(
    ball_motion_profile_t *profile,
    float target_position_m);
void ball_motion_profile_step(ball_motion_profile_t *profile, float dt_s);
const ball_motion_profile_output_t *ball_motion_profile_get_output(
    const ball_motion_profile_t *profile);

#endif
