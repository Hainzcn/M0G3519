#include "ball_return_demo_app.h"

#include <math.h>

#include "balance_actuator_trajectory.h"
#include "balance_linkage.h"
#include "button.h"
#include "control_config.h"
#include "control_config_legacy.h"
#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"

#define RETURN_DEMO_ADDRESS              (EMM42_DEFAULT_ADDRESS)
#define RETURN_DEMO_POWER_WAIT_MS        (3000u)
#define RETURN_DEMO_COMMAND_WAIT_MS      (100u)
#define RETURN_DEMO_LEVEL_TIMEOUT_MS     (2500u)
#define RETURN_DEMO_LEVEL_SETTLE_MS      (200u)
#define RETURN_DEMO_QUERY_PERIOD_MS      (20u)
#define RETURN_DEMO_INITIAL_POSITION_M   (0.050f)
#define RETURN_DEMO_TARGET_POSITION_M    (0.0f)
#define RETURN_DEMO_GRAVITY_MPS2         (9.80665f)
#define RETURN_DEMO_PI                   (3.14159265358979323846f)
#define RETURN_DEMO_RAD_TO_DEG           (180.0f / RETURN_DEMO_PI)

static ball_return_demo_state_enum return_demo_state;
static ball_motion_profile_t return_demo_profile;
static balance_actuator_trajectory_t return_demo_actuator;
static emm42_frame_t return_demo_rx_frame;
static uint32 return_demo_state_start_ms;
static uint32 return_demo_last_profile_ms;
static uint32 return_demo_last_outer_ms;
static uint32 return_demo_last_command_ms;
static uint32 return_demo_last_query_ms;
static uint32 return_demo_level_stable_start_ms;
static button_id_t return_demo_previous_button;
static float return_demo_level_motor_deg;
static float return_demo_motor_feedback_deg;
static float return_demo_motor_target_deg;
static float return_demo_last_sent_lever_deg;
static float return_demo_raw_lever_deg;
static uint8 return_demo_motor_feedback_valid;
static uint8 return_demo_level_stable;
static uint8 return_demo_has_sent_target;
static uint8 return_demo_profile_started;

static float return_demo_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float return_demo_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static uint8 return_demo_elapsed(uint32 now_ms, uint32 wait_ms)
{
    return ((now_ms - return_demo_state_start_ms) >= wait_ms) ? 1u : 0u;
}

static void return_demo_set_state(ball_return_demo_state_enum state,
                                  uint32 now_ms)
{
    return_demo_state = state;
    return_demo_state_start_ms = now_ms;
}

static void return_demo_fail(uint32 now_ms, const char *message)
{
    (void)emm42_stop(RETURN_DEMO_ADDRESS, 0u);
    heartbeat_hw_uart_send_string(message);
    return_demo_set_state(BALL_RETURN_DEMO_ERROR, now_ms);
}

static button_id_t return_demo_take_button_edge(void)
{
    button_id_t active = button_get_active();
    button_id_t edge = BUTTON_ID_NONE;
    if ((BUTTON_ID_NONE != active) &&
        (active != return_demo_previous_button))
        edge = active;
    return_demo_previous_button = active;
    return edge;
}

static void return_demo_drain_frames(void)
{
    float position_deg;
    while (0u != emm42_read_frame(&return_demo_rx_frame))
    {
        if (0u != emm42_decode_position_deg(
                &return_demo_rx_frame, RETURN_DEMO_ADDRESS, &position_deg))
        {
            return_demo_motor_feedback_deg = position_deg;
            return_demo_motor_feedback_valid = 1u;
        }
    }
}

static uint8 return_demo_send_lever(float lever_angle_deg, uint16 move_rpm,
                                   uint32 now_ms)
{
    float physical_angle =
        (float)BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN * lever_angle_deg;
    if (0u == balance_linkage_motor_from_physical_lever_deg(
            physical_angle, &return_demo_motor_target_deg))
        return 0u;
    if (0u == emm42_move_angle(RETURN_DEMO_ADDRESS,
            return_demo_motor_target_deg, move_rpm,
            BALANCE_EMM42_ACCELERATION, EMM42_POSITION_ABSOLUTE, 0u))
        return 0u;
    return_demo_last_command_ms = now_ms;
    return_demo_last_sent_lever_deg = lever_angle_deg;
    return_demo_has_sent_target = 1u;
    return 1u;
}

static float return_demo_inverse_dynamics(
    const ball_motion_profile_output_t *profile)
{
    float dynamics_accel;
    float inverse;

    if ((BALL_MOTION_PHASE_CAPTURE == profile->phase) ||
        (BALL_MOTION_PHASE_HOLD == profile->phase))
        return 0.0f;
    dynamics_accel = profile->feedforward_accel_mps2;
    if (return_demo_abs(profile->velocity_mps) > BALANCE_STICK_VELOCITY_MPS)
    {
        dynamics_accel += (profile->velocity_mps < 0.0f) ?
            -BALANCE_ROLLING_FRICTION_ACCEL_MPS2 :
             BALANCE_ROLLING_FRICTION_ACCEL_MPS2;
    }
    inverse = -dynamics_accel /
        (BALANCE_ROLLING_FACTOR * RETURN_DEMO_GRAVITY_MPS2);
    inverse = return_demo_clamp(inverse, -1.0f, 1.0f);
    return return_demo_clamp(asinf(inverse) * RETURN_DEMO_RAD_TO_DEG,
        -BALANCE_MAX_LEVER_ANGLE_DEG, BALANCE_MAX_LEVER_ANGLE_DEG);
}

static void return_demo_begin_run(uint32 now_ms)
{
    ball_motion_profile_reset(&return_demo_profile,
                              RETURN_DEMO_INITIAL_POSITION_M, 0.0f);
    ball_motion_profile_set_target(&return_demo_profile,
                                   RETURN_DEMO_TARGET_POSITION_M);
    balance_actuator_trajectory_reset(&return_demo_actuator, 0.0f);
    return_demo_raw_lever_deg = 0.0f;
    return_demo_has_sent_target = 0u;
    return_demo_profile_started = 0u;
    return_demo_last_profile_ms = now_ms;
    return_demo_last_outer_ms = now_ms;
    return_demo_last_command_ms = now_ms - BALANCE_COMMAND_PERIOD_MS;
    heartbeat_hw_uart_send_string(
        "[ball-return] SW1 start: assumed x=+50mm, returning center\r\n");
    return_demo_set_state(BALL_RETURN_DEMO_RUNNING, now_ms);
}

static void return_demo_run_control(uint32 now_ms)
{
    const ball_motion_profile_output_t *profile;
    const balance_actuator_trajectory_output_t *actuator;

    if ((now_ms - return_demo_last_profile_ms) >= BALANCE_ESTIMATOR_PERIOD_MS)
    {
        return_demo_last_profile_ms += BALANCE_ESTIMATOR_PERIOD_MS;
        if ((now_ms - return_demo_last_profile_ms) >= BALANCE_ESTIMATOR_PERIOD_MS)
            return_demo_last_profile_ms = now_ms;
        ball_motion_profile_step(&return_demo_profile,
            (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f *
            BALL_RETURN_DEMO_SPEED_SCALE);
        return_demo_profile_started = 1u;
    }
    profile = ball_motion_profile_get_output(&return_demo_profile);
    if ((now_ms - return_demo_last_outer_ms) >= BALANCE_OUTER_CONTROL_PERIOD_MS)
    {
        return_demo_last_outer_ms += BALANCE_OUTER_CONTROL_PERIOD_MS;
        if ((now_ms - return_demo_last_outer_ms) >= BALANCE_OUTER_CONTROL_PERIOD_MS)
            return_demo_last_outer_ms = now_ms;
        return_demo_raw_lever_deg = return_demo_inverse_dynamics(profile);
        balance_actuator_trajectory_step(&return_demo_actuator,
            return_demo_raw_lever_deg,
            (float)BALANCE_OUTER_CONTROL_PERIOD_MS * 0.001f);
    }
    actuator = balance_actuator_trajectory_get_output(&return_demo_actuator);
    if (((now_ms - return_demo_last_command_ms) >= BALANCE_COMMAND_PERIOD_MS) &&
        ((0u == return_demo_has_sent_target) ||
         (return_demo_abs(actuator->angle_deg -
                          return_demo_last_sent_lever_deg) >=
          BALANCE_LEVER_COMMAND_DEADBAND_DEG) ||
         (0.0f == actuator->angle_deg)))
    {
        if (0u == return_demo_send_lever(
                actuator->angle_deg, BALANCE_EMM42_MOVE_RPM, now_ms))
        {
            return_demo_fail(now_ms,
                "[ball-return] actuator command failed\r\n");
            return;
        }
    }
    if ((0u != return_demo_profile_started) &&
        ((BALL_MOTION_PHASE_CAPTURE == profile->phase) ||
         (BALL_MOTION_PHASE_HOLD == profile->phase)))
    {
        heartbeat_hw_uart_send_string(
            "[ball-return] capture reached; leveling lever\r\n");
        return_demo_set_state(BALL_RETURN_DEMO_SETTLING, now_ms);
    }
}

static void return_demo_settle(uint32 now_ms)
{
    const balance_actuator_trajectory_output_t *actuator;
    if ((now_ms - return_demo_last_outer_ms) >= BALANCE_OUTER_CONTROL_PERIOD_MS)
    {
        return_demo_last_outer_ms = now_ms;
        balance_actuator_trajectory_step(&return_demo_actuator, 0.0f,
            (float)BALANCE_OUTER_CONTROL_PERIOD_MS * 0.001f);
    }
    actuator = balance_actuator_trajectory_get_output(&return_demo_actuator);
    if ((now_ms - return_demo_last_command_ms) >= BALANCE_COMMAND_PERIOD_MS)
    {
        if (0u == return_demo_send_lever(
                actuator->angle_deg, BALANCE_EMM42_MOVE_RPM, now_ms))
        {
            return_demo_fail(now_ms,
                "[ball-return] level command failed\r\n");
            return;
        }
    }
    if ((return_demo_abs(actuator->angle_deg) < 0.001f) &&
        (return_demo_abs(actuator->rate_deg_s) < 0.001f))
    {
        heartbeat_hw_uart_send_string(
            "[ball-return] complete; SW1 repeats, SW4 stops\r\n");
        return_demo_set_state(BALL_RETURN_DEMO_DONE, now_ms);
    }
}

void ball_return_demo_app_init(void)
{
    uint32 now_ms = heartbeat_get_ms();
    ball_motion_profile_config_t profile_config;
    balance_actuator_trajectory_config_t actuator_config;

    profile_config.drive_accel_mps2 = BALANCE_PROFILE_DRIVE_ACCEL_MPS2;
    profile_config.brake_accel_mps2 = BALANCE_PROFILE_BRAKE_ACCEL_MPS2;
    profile_config.max_velocity_mps = BALANCE_PROFILE_MAX_VELOCITY_MPS;
    profile_config.max_jerk_mps3 = BALANCE_PROFILE_MAX_JERK_MPS3;
    profile_config.feedforward_lead_s = BALANCE_PROFILE_FEEDFORWARD_LEAD_S;
    profile_config.capture_position_m = BALANCE_PROFILE_CAPTURE_POSITION_M;
    profile_config.capture_velocity_mps = BALANCE_PROFILE_CAPTURE_VELOCITY_MPS;
    profile_config.position_tolerance_m =
        BALANCE_PROFILE_POSITION_TOLERANCE_M;
    profile_config.velocity_tolerance_mps =
        BALANCE_PROFILE_VELOCITY_TOLERANCE_MPS;
    ball_motion_profile_init(&return_demo_profile, &profile_config);

    actuator_config.max_angle_deg = BALANCE_MAX_LEVER_ANGLE_DEG;
    actuator_config.max_rate_deg_s = BALANCE_MAX_LEVER_RATE_DEG_S;
    actuator_config.max_accel_deg_s2 = BALANCE_MAX_LEVER_ACCEL_DEG_S2;
    balance_actuator_trajectory_init(&return_demo_actuator, &actuator_config);

    return_demo_state = BALL_RETURN_DEMO_WAIT_POWER;
    return_demo_state_start_ms = now_ms;
    return_demo_last_profile_ms = now_ms;
    return_demo_last_outer_ms = now_ms;
    return_demo_last_command_ms = now_ms;
    return_demo_last_query_ms = now_ms;
    return_demo_level_stable_start_ms = now_ms;
    return_demo_previous_button = BUTTON_ID_NONE;
    return_demo_level_motor_deg = 0.0f;
    return_demo_motor_feedback_deg = 0.0f;
    return_demo_motor_target_deg = 0.0f;
    return_demo_last_sent_lever_deg = 0.0f;
    return_demo_raw_lever_deg = 0.0f;
    return_demo_motor_feedback_valid = 0u;
    return_demo_level_stable = 0u;
    return_demo_has_sent_target = 0u;
    return_demo_profile_started = 0u;
    return_demo_rx_frame.length = 0u;
    emm42_init();
    if (BALL_RETURN_DEMO_SPEED_SCALE <= 0.0f)
    {
        heartbeat_hw_uart_send_string(
            "[ball-return] invalid speed scale; must be > 0\r\n");
        return_demo_state = BALL_RETURN_DEMO_ERROR;
        return;
    }
    heartbeat_hw_uart_send_string(
        "[ball-return] lower stop required; leveling after 3s\r\n");
}

void ball_return_demo_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();
    button_id_t button_edge;

    return_demo_drain_frames();
    button_edge = return_demo_take_button_edge();
    if ((BUTTON_ID_SW4 == button_edge) &&
        (BALL_RETURN_DEMO_ERROR != return_demo_state))
        return_demo_fail(now_ms, "[ball-return] emergency stop\r\n");

    switch (return_demo_state)
    {
        case BALL_RETURN_DEMO_WAIT_POWER:
            if (return_demo_elapsed(now_ms, RETURN_DEMO_POWER_WAIT_MS))
            {
                if (0u == emm42_set_current_position_zero(RETURN_DEMO_ADDRESS))
                    return_demo_fail(now_ms, "[ball-return] zero failed\r\n");
                else
                    return_demo_set_state(BALL_RETURN_DEMO_WAIT_ZERO, now_ms);
            }
            break;
        case BALL_RETURN_DEMO_WAIT_ZERO:
            if (return_demo_elapsed(now_ms, RETURN_DEMO_COMMAND_WAIT_MS))
            {
                if (0u == emm42_set_enabled(RETURN_DEMO_ADDRESS, 1u, 0u))
                    return_demo_fail(now_ms, "[ball-return] enable failed\r\n");
                else
                    return_demo_set_state(BALL_RETURN_DEMO_WAIT_ENABLE, now_ms);
            }
            break;
        case BALL_RETURN_DEMO_WAIT_ENABLE:
            if (return_demo_elapsed(now_ms, RETURN_DEMO_COMMAND_WAIT_MS))
                return_demo_set_state(BALL_RETURN_DEMO_MOVE_LEVEL, now_ms);
            break;
        case BALL_RETURN_DEMO_MOVE_LEVEL:
            if ((0u == balance_linkage_motor_from_physical_lever_deg(
                    0.0f, &return_demo_level_motor_deg)) ||
                (0u == return_demo_send_lever(
                    0.0f, BALANCE_LEVEL_RETURN_RPM, now_ms)))
                return_demo_fail(now_ms, "[ball-return] level move failed\r\n");
            else
            {
                return_demo_motor_feedback_valid = 0u;
                return_demo_last_query_ms = now_ms;
                return_demo_level_stable = 0u;
                return_demo_set_state(BALL_RETURN_DEMO_WAIT_LEVEL, now_ms);
            }
            break;
        case BALL_RETURN_DEMO_WAIT_LEVEL:
            if (return_demo_elapsed(now_ms, RETURN_DEMO_LEVEL_TIMEOUT_MS))
                return_demo_fail(now_ms, "[ball-return] level timeout\r\n");
            else if ((0u != return_demo_motor_feedback_valid) &&
                     (return_demo_abs(return_demo_motor_feedback_deg -
                                      return_demo_level_motor_deg) <=
                      BALANCE_LEVEL_MOTOR_TOLERANCE_DEG))
            {
                if (0u == return_demo_level_stable)
                {
                    return_demo_level_stable = 1u;
                    return_demo_level_stable_start_ms = now_ms;
                }
                else if ((now_ms - return_demo_level_stable_start_ms) >=
                         RETURN_DEMO_LEVEL_SETTLE_MS)
                {
                    heartbeat_hw_uart_send_string(
                        "[ball-return] ready: place ball at +5cm, press SW1\r\n");
                    return_demo_set_state(BALL_RETURN_DEMO_READY, now_ms);
                }
            }
            else
                return_demo_level_stable = 0u;
            if ((now_ms - return_demo_last_query_ms) >=
                RETURN_DEMO_QUERY_PERIOD_MS)
            {
                return_demo_last_query_ms = now_ms;
                (void)emm42_query_position(RETURN_DEMO_ADDRESS);
            }
            break;
        case BALL_RETURN_DEMO_READY:
        case BALL_RETURN_DEMO_DONE:
            if (BUTTON_ID_SW1 == button_edge) return_demo_begin_run(now_ms);
            break;
        case BALL_RETURN_DEMO_RUNNING:
            return_demo_run_control(now_ms);
            break;
        case BALL_RETURN_DEMO_SETTLING:
            return_demo_settle(now_ms);
            break;
        case BALL_RETURN_DEMO_ERROR:
        default:
            break;
    }
}

ball_return_demo_state_enum ball_return_demo_app_get_state(void)
{
    return return_demo_state;
}

float ball_return_demo_app_get_reference_position_m(void)
{
    return return_demo_profile.output.position_m;
}

float ball_return_demo_app_get_reference_velocity_mps(void)
{
    return return_demo_profile.output.velocity_mps;
}

float ball_return_demo_app_get_raw_lever_angle_deg(void)
{
    return return_demo_raw_lever_deg;
}

float ball_return_demo_app_get_lever_angle_deg(void)
{
    return return_demo_actuator.output.angle_deg;
}

float ball_return_demo_app_get_motor_target_deg(void)
{
    return return_demo_motor_target_deg;
}

ball_motion_phase_enum ball_return_demo_app_get_motion_phase(void)
{
    return return_demo_profile.output.phase;
}
