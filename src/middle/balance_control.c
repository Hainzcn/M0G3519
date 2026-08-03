#include "balance_control.h"

#include <math.h>

#define CONTROL_GRAVITY_MPS2       (9.80665f)
#define CONTROL_PI                 (3.14159265358979323846f)
#define CONTROL_DEG_TO_RAD         (CONTROL_PI / 180.0f)
#define CONTROL_RAD_TO_DEG         (180.0f / CONTROL_PI)
#define CONTROL_POSITION_LIMIT_M   (0.13f)
#define CONTROL_VELOCITY_LIMIT_MPS (5.0f)

static float control_abs(float value) { return (value < 0.0f) ? -value : value; }
static float control_sign(float value) { return (value < 0.0f) ? -1.0f : 1.0f; }

static float control_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

float balance_control_vehicle_sync_lever_deg(float car_accel_mps2)
{
    return -atan2f(car_accel_mps2, CONTROL_GRAVITY_MPS2) *
        CONTROL_RAD_TO_DEG;
}

static float control_friction_scale(const balance_control_t *control,
                                    float velocity_mps)
{
    if (control->config.stick_velocity_mps <= 0.000001f) return 1.0f;
    return control_clamp(
        (control_abs(velocity_mps) - control->config.stick_velocity_mps) /
        control->config.stick_velocity_mps, 0.0f, 1.0f);
}

static void control_clear_overspeed(balance_control_t *control)
{
    control->overspeed_active = 0u;
    control->overspeed_hold_remaining_ms = 0u;
    control->overspeed_accel_sign = 0.0f;
    control->overspeed_target_position_m = 0.0f;
}

static void control_clear_stiction(balance_control_t *control)
{
    control->stuck_elapsed_ms = 0u;
    control->breakaway_remaining_ms = 0u;
    control->stuck_anchor_position_m = 0.0f;
    control->stuck_anchor_valid = 0u;
}

static float control_model_accel(const balance_control_t *control,
                                 float lever_deg, float position_m,
                                 float velocity_mps,
                                 float car_accel_mps2)
{
    float lever_rad = lever_deg * CONTROL_DEG_TO_RAD +
        control->config.rail_curvature_m_inv * position_m;
    float accel = -control->config.rolling_factor *
        (CONTROL_GRAVITY_MPS2 * sinf(lever_rad) +
         car_accel_mps2 * cosf(lever_rad));
    if (control_abs(velocity_mps) > control->config.stick_velocity_mps)
    {
        accel -= control_sign(velocity_mps) *
                 control->config.rolling_friction_accel_mps2 *
                 control_friction_scale(control, velocity_mps);
    }
    return accel;
}

static void control_integrate(const balance_control_t *control,
                              float lever_deg, float car_accel_mps2,
                              float dt_s, float *position_m,
                              float *velocity_mps)
{
    float accel = control_model_accel(control, lever_deg, *position_m,
                                      *velocity_mps, car_accel_mps2);
    *position_m += *velocity_mps * dt_s + 0.5f * accel * dt_s * dt_s;
    *velocity_mps += accel * dt_s;
    *position_m = control_clamp(*position_m, -CONTROL_POSITION_LIMIT_M,
                                CONTROL_POSITION_LIMIT_M);
    *velocity_mps = control_clamp(*velocity_mps,
                                  -CONTROL_VELOCITY_LIMIT_MPS,
                                  CONTROL_VELOCITY_LIMIT_MPS);
}

static void control_record_command(balance_control_t *control, float angle_deg)
{
    control->command_history_head = (uint8)(
        (control->command_history_head + 1u) %
        BALANCE_CONTROL_COMMAND_HISTORY_COUNT);
    control->command_history[control->command_history_head] = angle_deg;
    if (control->command_history_count < BALANCE_CONTROL_COMMAND_HISTORY_COUNT)
    {
        control->command_history_count++;
    }
}

static float control_history_age(const balance_control_t *control,
                                 uint8 age, uint8 *degraded)
{
    uint8 available_age;
    uint8 index;
    if (0u == control->command_history_count)
    {
        *degraded = 1u;
        return 0.0f;
    }
    available_age = (uint8)(control->command_history_count - 1u);
    if (age > available_age)
    {
        age = available_age;
        *degraded = 1u;
    }
    index = (uint8)((control->command_history_head +
        BALANCE_CONTROL_COMMAND_HISTORY_COUNT - age) %
        BALANCE_CONTROL_COMMAND_HISTORY_COUNT);
    return control->command_history[index];
}

static uint8 control_delay_steps(const balance_control_t *control)
{
    float steps;
    if (control->config.command_period_s <= 0.0f) return 0u;
    steps = control->config.actuator_delay_s /
            control->config.command_period_s;
    if (steps > (float)(BALANCE_CONTROL_COMMAND_HISTORY_COUNT - 1u))
        steps = (float)(BALANCE_CONTROL_COMMAND_HISTORY_COUNT - 1u);
    return (uint8)(steps + 0.5f);
}

static void control_predict(balance_control_t *control,
                            float car_accel_mps2, uint16 *flags)
{
    uint8 delay_steps = control_delay_steps(control);
    uint8 degraded = 0u;
    uint8 age;
    float remaining;
    float step_s;

    control->output.predicted_position_m = control->output.estimated_position_m;
    control->output.predicted_velocity_mps = control->output.estimated_velocity_mps;
    remaining = control->config.actuator_delay_s;
    age = delay_steps;
    while (remaining > 0.000001f)
    {
        step_s = (remaining < control->config.command_period_s) ?
            remaining : control->config.command_period_s;
        control_integrate(control,
            control_history_age(control, age, &degraded), car_accel_mps2,
            step_s, &control->output.predicted_position_m,
            &control->output.predicted_velocity_mps);
        remaining -= step_s;
        if (age > 0u) age--;
    }
    if (0u != degraded) *flags |= BALANCE_CONTROL_FLAG_PREDICTOR_DEGRADED;
}

void balance_control_init(balance_control_t *control,
                          const balance_control_config_t *config)
{
    if ((NULL == control) || (NULL == config)) return;
    control->config = *config;
    balance_control_reset(control);
}

void balance_control_reset(balance_control_t *control)
{
    uint8 index;
    if (NULL == control) return;
    control->output.has_state = 0u;
    control->output.flags = BALANCE_CONTROL_FLAG_PREDICT_ONLY;
    control->output.estimated_position_m = 0.0f;
    control->output.estimated_velocity_mps = 0.0f;
    control->output.predicted_position_m = 0.0f;
    control->output.predicted_velocity_mps = 0.0f;
    control->output.position_error_m = 0.0f;
    control->output.velocity_command_mps = 0.0f;
    control->output.velocity_limit_mps = 0.0f;
    control->output.brake_distance_m = 0.0f;
    control->output.feedforward_accel_mps2 = 0.0f;
    control->output.feedback_accel_mps2 = 0.0f;
    control->output.desired_ball_accel_mps2 = 0.0f;
    control->output.lever_angle_deg = 0.0f;
    control->output.phase = BALANCE_CONTROL_PHASE_HOLD;
    control->output.friction_mode = BALANCE_FRICTION_STOPPED;
    control->command_history_head = 0u;
    control->command_history_count = 0u;
    for (index = 0u; index < BALANCE_CONTROL_COMMAND_HISTORY_COUNT; index++)
        control->command_history[index] = 0.0f;
    control_clear_stiction(control);
    control_clear_overspeed(control);
    control->capture_integral = 0.0f;
}

void balance_control_step(balance_control_t *control,
                          const balance_control_input_t *input)
{
    balance_control_output_t *output;
    uint16 flags = 0u;
    uint8 degraded = 0u;
    float lever_for_model;
    float residual;
    float target_error;
    float reference_error;
    float direction;
    float available;
    float desired_accel = 0.0f;
    float feedback_accel = 0.0f;
    float velocity_command = 0.0f;
    float velocity_limit = 0.0f;
    float dynamics_accel;
    float dynamics_limit;
    float radius;
    float lever_deg = 0.0f;
    float shape_angle_deg;
    float angle_limit;
    float velocity_toward_target;
    uint32 outer_ms;

    if ((NULL == control) || (NULL == input) || (input->dt_s <= 0.0f)) return;
    output = &control->output;
    if (0u != input->actuator_command_updated)
        control_record_command(control, input->actuator_command_angle_deg);

    if (0u != output->has_state)
    {
        lever_for_model = (0u != input->actual_lever_valid) ?
            input->actual_lever_angle_deg :
            control_history_age(control, control_delay_steps(control), &degraded);
        control_integrate(control, lever_for_model, input->car_accel_mps2,
                          input->dt_s, &output->estimated_position_m,
                          &output->estimated_velocity_mps);
        if (0u != degraded) flags |= BALANCE_CONTROL_FLAG_PREDICTOR_DEGRADED;
    }

    if ((0u != input->new_measurement) && (0u != input->measurement_valid))
    {
        if (0u == output->has_state)
        {
            output->estimated_position_m = input->measured_position_m;
            output->estimated_velocity_mps = input->measured_velocity_mps;
            output->has_state = 1u;
        }
        else
        {
            residual = input->measured_position_m - output->estimated_position_m;
            output->estimated_position_m +=
                control->config.position_correction_gain * residual;
            output->estimated_velocity_mps +=
                control->config.velocity_residual_gain *
                (input->measured_velocity_mps -
                 output->estimated_velocity_mps);
        }
    }

    if ((0u != input->measurement_valid) &&
        (input->measurement_age_ms <= control->config.fresh_measurement_ms))
        flags |= BALANCE_CONTROL_FLAG_MEASUREMENT_FRESH;
    else
        flags |= BALANCE_CONTROL_FLAG_PREDICT_ONLY;
    if (0u != control->config.calibration_provisional)
        flags |= BALANCE_CONTROL_FLAG_CALIBRATION_PENDING;

    if (0u != output->has_state)
        control_predict(control, input->car_accel_mps2, &flags);
    else
    {
        output->predicted_position_m = 0.0f;
        output->predicted_velocity_mps = 0.0f;
    }
    output->position_error_m = input->target_position_m -
                               output->predicted_position_m;
    shape_angle_deg = control->config.rail_curvature_m_inv *
        output->predicted_position_m * CONTROL_RAD_TO_DEG;
    if (0u == input->update_control_output)
    {
        output->flags = flags;
        return;
    }

    output->phase = BALANCE_CONTROL_PHASE_HOLD;
    output->friction_mode = BALANCE_FRICTION_MOTION;
    if ((0u == output->has_state) ||
        (input->measurement_age_ms > control->config.valid_measurement_ms))
    {
        control->capture_integral = 0.0f;
        control_clear_stiction(control);
        control_clear_overspeed(control);
        output->friction_mode = BALANCE_FRICTION_STOPPED;
        goto inverse_dynamics;
    }

    target_error = input->target_position_m - output->predicted_position_m;
    reference_error = input->reference_position_m - output->predicted_position_m;
    direction = control_sign(target_error);
    velocity_toward_target = output->predicted_velocity_mps * direction;
    output->brake_distance_m =
        output->predicted_velocity_mps * output->predicted_velocity_mps /
        (2.0f * control->config.brake_accel_mps2);
    available = control_abs(target_error) -
        control_abs(output->predicted_velocity_mps) *
        control->config.brake_margin_delay_s;
    if (available < 0.0f) available = 0.0f;
    velocity_limit = sqrtf(2.0f * control->config.brake_accel_mps2 * available);
    if (velocity_limit > control->config.max_ball_velocity_mps)
        velocity_limit = control->config.max_ball_velocity_mps;

    if ((control_abs(target_error) <= control->config.center_dead_position_m) &&
        (control_abs(output->predicted_velocity_mps) <=
         control->config.stick_velocity_mps))
    {
        control->capture_integral = 0.0f;
        control_clear_stiction(control);
        control_clear_overspeed(control);
        output->friction_mode = BALANCE_FRICTION_STOPPED;
        output->phase = BALANCE_CONTROL_PHASE_HOLD;
    }
    else if ((control_abs(target_error) <= control->config.capture_position_m) &&
             (control_abs(output->predicted_velocity_mps) <=
              control->config.capture_velocity_mps))
    {
        control->capture_integral += target_error * control->config.command_period_s;
        feedback_accel = control_clamp(
            control->config.capture_integral_gain * control->capture_integral,
            -control->config.capture_max_accel_mps2,
            control->config.capture_max_accel_mps2);
        desired_accel = feedback_accel;
        control_clear_stiction(control);
        control_clear_overspeed(control);
        output->friction_mode = BALANCE_FRICTION_CAPTURE;
        output->phase = BALANCE_CONTROL_PHASE_CAPTURE;
        flags |= BALANCE_CONTROL_FLAG_CAPTURE_ACTIVE;
    }
    else
    {
        uint32 measurement_ms;
        control->capture_integral = 0.0f;
        outer_ms = (uint32)(control->config.command_period_s * 1000.0f + 0.5f);
        if ((0u != input->new_measurement) &&
            (0u != input->measurement_valid))
        {
            measurement_ms = (uint32)(input->measurement_interval_s *
                                      1000.0f + 0.5f);
            if (0u == measurement_ms) measurement_ms = outer_ms;
            if (0u == control->stuck_anchor_valid)
            {
                control->stuck_anchor_position_m = input->measured_position_m;
                control->stuck_anchor_valid = 1u;
                control->stuck_elapsed_ms = (measurement_ms >=
                    control->config.breakaway_qualify_ms) ?
                    control->config.breakaway_qualify_ms : measurement_ms;
            }
            else if (control_abs(input->measured_position_m -
                                 control->stuck_anchor_position_m) <=
                     control->config.breakaway_movement_m)
            {
                if (measurement_ms >= control->config.breakaway_qualify_ms -
                    control->stuck_elapsed_ms)
                    control->stuck_elapsed_ms =
                        control->config.breakaway_qualify_ms;
                else
                    control->stuck_elapsed_ms += measurement_ms;
            }
            else
            {
                control->stuck_elapsed_ms = 0u;
                control->breakaway_remaining_ms = 0u;
                control->stuck_anchor_position_m = input->measured_position_m;
            }
        }
        if ((0u == control->breakaway_remaining_ms) &&
            (control->stuck_elapsed_ms >= control->config.breakaway_qualify_ms))
        {
            control->breakaway_remaining_ms = control->config.breakaway_pulse_ms;
            control->stuck_elapsed_ms = 0u;
            control->stuck_anchor_valid = 0u;
        }
        if (control->breakaway_remaining_ms > 0u)
        {
            lever_deg = -direction * control->config.breakaway_angle_deg -
                        shape_angle_deg;
            control->breakaway_remaining_ms =
                (control->breakaway_remaining_ms > outer_ms) ?
                control->breakaway_remaining_ms - outer_ms : 0u;
            output->friction_mode = BALANCE_FRICTION_BREAKAWAY;
            output->phase = BALANCE_CONTROL_PHASE_ACCEL;
            flags |= BALANCE_CONTROL_FLAG_BREAKAWAY_ACTIVE;
            control_clear_overspeed(control);
            goto finalize;
        }
        if (control_abs(output->predicted_position_m) >=
            control->config.edge_position_m)
        {
            control_clear_overspeed(control);
            desired_accel = (output->predicted_position_m > 0.0f) ?
                -control->config.edge_recovery_accel_mps2 :
                control->config.edge_recovery_accel_mps2;
            feedback_accel = desired_accel;
            output->phase = BALANCE_CONTROL_PHASE_EDGE_RECOVERY;
            flags |= BALANCE_CONTROL_FLAG_EDGE_RECOVERY;
        }
        else
        {
            /* Hold one pullback direction across noisy braking boundaries. */
            if ((0u != control->overspeed_active) &&
                (control_abs(input->target_position_m -
                             control->overspeed_target_position_m) >
                 control->config.center_dead_position_m))
            {
                control_clear_overspeed(control);
            }
            if (0u != control->overspeed_active)
            {
                if (control->overspeed_hold_remaining_ms > outer_ms)
                    control->overspeed_hold_remaining_ms -= outer_ms;
                else
                    control->overspeed_hold_remaining_ms = 0u;

                if (0u == control->overspeed_hold_remaining_ms)
                {
                    uint8 release_overspeed = 0u;
                    if ((control->overspeed_accel_sign * direction) > 0.0f)
                    {
                        if (velocity_toward_target >=
                            control->config.stick_velocity_mps)
                            release_overspeed = 1u;
                    }
                    else if ((velocity_toward_target <=
                              control->config.stick_velocity_mps) ||
                             (output->brake_distance_m <=
                              control->config.overspeed_release_ratio *
                              available))
                    {
                        release_overspeed = 1u;
                    }
                    if (0u != release_overspeed)
                        control_clear_overspeed(control);
                }
            }
            if ((0u == control->overspeed_active) &&
                ((velocity_toward_target <
                  -control->config.stick_velocity_mps) ||
                 ((velocity_toward_target > 0.0f) &&
                  (output->brake_distance_m >= available))))
            {
                control->overspeed_active = 1u;
                control->overspeed_hold_remaining_ms =
                    control->config.overspeed_min_hold_ms;
                control->overspeed_accel_sign =
                    (velocity_toward_target < 0.0f) ? direction :
                    -control_sign(output->predicted_velocity_mps);
                control->overspeed_target_position_m =
                    input->target_position_m;
            }
            if (0u != control->overspeed_active)
            {
                desired_accel = control->overspeed_accel_sign *
                    control->config.brake_accel_mps2;
                feedback_accel = desired_accel;
                output->phase = BALANCE_CONTROL_PHASE_OVERSPEED;
                flags |= BALANCE_CONTROL_FLAG_OVERSPEED_PULLBACK;
            }
            else
            {
                float unclamped_velocity_command;
                velocity_command = input->reference_velocity_mps +
                    control->config.position_gain_s_inv * reference_error;
                unclamped_velocity_command = velocity_command;
                velocity_command = control_clamp(velocity_command,
                                                  -velocity_limit,
                                                  velocity_limit);
                if (control_abs(unclamped_velocity_command - velocity_command) >
                    0.000001f)
                    flags |= BALANCE_CONTROL_FLAG_VELOCITY_SATURATED;
                feedback_accel = control->config.velocity_gain_s_inv *
                    (velocity_command - output->predicted_velocity_mps);
                desired_accel = input->feedforward_accel_mps2 + feedback_accel;
                output->phase = (control_abs(input->feedforward_accel_mps2) >
                    0.001f) ? BALANCE_CONTROL_PHASE_ACCEL :
                              BALANCE_CONTROL_PHASE_TRACK;
            }
        }
    }

inverse_dynamics:
    if (control_abs(desired_accel) > control->config.max_ball_accel_mps2)
    {
        desired_accel = control_clamp(desired_accel,
            -control->config.max_ball_accel_mps2,
            control->config.max_ball_accel_mps2);
        flags |= BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED;
    }
    dynamics_accel = desired_accel;
    if ((BALANCE_FRICTION_MOTION == output->friction_mode) &&
        (control_abs(output->predicted_velocity_mps) >
         control->config.stick_velocity_mps))
        dynamics_accel += control_sign(output->predicted_velocity_mps) *
            control->config.rolling_friction_accel_mps2 *
            control_friction_scale(control, output->predicted_velocity_mps);
    radius = sqrtf(CONTROL_GRAVITY_MPS2 * CONTROL_GRAVITY_MPS2 +
                   input->car_accel_mps2 * input->car_accel_mps2);
    dynamics_limit = -dynamics_accel /
        (control->config.rolling_factor * radius);
    if (dynamics_limit > 1.0f)
    {
        dynamics_limit = 1.0f;
        flags |= BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED;
    }
    else if (dynamics_limit < -1.0f)
    {
        dynamics_limit = -1.0f;
        flags |= BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED;
    }
    /*
     * With no ball/rail relative motion, tan(theta) = -a_car/g. The asin
     * term is only the relative ball-position correction around that angle.
     */
    lever_deg = asinf(dynamics_limit) * CONTROL_RAD_TO_DEG +
        balance_control_vehicle_sync_lever_deg(input->car_accel_mps2) -
        shape_angle_deg;

finalize:
    angle_limit = (input->measurement_age_ms >
        control->config.fresh_measurement_ms) ?
        control->config.degraded_lever_angle_deg :
        control->config.max_lever_angle_deg;
    if (control_abs(lever_deg) > angle_limit)
    {
        lever_deg = control_clamp(lever_deg, -angle_limit, angle_limit);
        flags |= BALANCE_CONTROL_FLAG_ANGLE_SATURATED;
    }
    if (control_abs(output->predicted_position_m) >=
        control->config.hard_edge_position_m)
        flags |= BALANCE_CONTROL_FLAG_HARD_EDGE;
    output->velocity_command_mps = velocity_command;
    output->velocity_limit_mps = velocity_limit;
    output->feedforward_accel_mps2 = input->feedforward_accel_mps2;
    output->feedback_accel_mps2 = feedback_accel;
    output->desired_ball_accel_mps2 = desired_accel;
    output->lever_angle_deg = lever_deg;
    output->flags = flags;
}

const balance_control_output_t *balance_control_get_output(
    const balance_control_t *control)
{
    return (NULL == control) ? NULL : &control->output;
}
