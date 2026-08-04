#include "no_load_lap_app.h"

#include <stdio.h>

#include "control_config.h"
#include "encoder.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "line_control.h"
#include "motor_app.h"

#define NO_LOAD_LAP_PI_F                  (3.14159265f)

static no_load_lap_status_t no_load_status;
static uint32 no_load_start_ms;
static int32 no_load_previous_left_count;
static int32 no_load_previous_right_count;
static uint32 no_load_marker_start_ms;
static uint32 no_load_line_loss_start_ms;
static uint32 no_load_sensor_offline_start_ms;
static float no_load_marker_distance_m;
static uint8 no_load_marker_active;
static uint8 no_load_line_loss_active;
static uint8 no_load_sensor_offline_active;

static void no_load_update_distance(void)
{
    int32 left_count = encoder_get_left_total_count();
    int32 right_count = encoder_get_right_total_count();
    int32 left_delta = left_count - no_load_previous_left_count;
    int32 right_delta = right_count - no_load_previous_right_count;
    float center_counts;
    float distance_delta_m;

    no_load_previous_left_count = left_count;
    no_load_previous_right_count = right_count;
    center_counts = 0.5f *
        ((float)left_delta * WHEEL_LEFT_ENCODER_SIGN +
         (float)right_delta * WHEEL_RIGHT_ENCODER_SIGN);
    distance_delta_m = center_counts *
        (NO_LOAD_LAP_PI_F * CHASSIS_WHEEL_DIAMETER_M) /
        (float)ENCODER_COUNTS_PER_WHEEL_REV;
    if (distance_delta_m > 0.0f)
    {
        no_load_status.distance_m += distance_delta_m;
    }
}

static void no_load_log_result(void)
{
    char message[96];
    uint32 distance_mm = (uint32)(no_load_status.distance_m * 1000.0f);

    snprintf(message, sizeof(message),
        "[no-load] end=%u,t=%lu,dist=%lu\r\n",
        (unsigned int)no_load_status.state,
        (unsigned long)no_load_status.elapsed_ms,
        (unsigned long)distance_mm);
    heartbeat_hw_uart_send_string(message);
}

static void no_load_finish(no_load_lap_state_enum state, uint32 now_ms)
{
    motor_app_stop();
    motor_app_set_base_rpm(NO_LOAD_LAP_CRUISE_RPM);
    no_load_status.elapsed_ms = now_ms - no_load_start_ms;
    no_load_status.state = state;
    no_load_marker_active = 0u;
    no_load_line_loss_active = 0u;
    no_load_sensor_offline_active = 0u;
    no_load_log_result();
}

void no_load_lap_app_init(void)
{
    no_load_status.state = NO_LOAD_LAP_IDLE;
    no_load_status.finish_armed = 0u;
    no_load_status.approach_active = 0u;
    no_load_status.brake_active = 0u;
    no_load_status.elapsed_ms = 0u;
    no_load_status.distance_m = 0.0f;
    no_load_status.brake_distance_m = 0.0f;
    no_load_start_ms = heartbeat_get_ms();
    no_load_previous_left_count = encoder_get_left_total_count();
    no_load_previous_right_count = encoder_get_right_total_count();
    no_load_marker_start_ms = 0u;
    no_load_line_loss_start_ms = 0u;
    no_load_sensor_offline_start_ms = 0u;
    no_load_marker_distance_m = 0.0f;
    no_load_marker_active = 0u;
    no_load_line_loss_active = 0u;
    no_load_sensor_offline_active = 0u;
}

uint8 no_load_lap_app_start(void)
{
    uint32 now_ms = heartbeat_get_ms();

    if (MOTOR_APP_MODE_DISABLED != motor_app_get_mode())
    {
        return 0u;
    }

    no_load_status.state = NO_LOAD_LAP_RUNNING;
    no_load_status.finish_armed = 0u;
    no_load_status.approach_active = 0u;
    no_load_status.brake_active = 0u;
    no_load_status.elapsed_ms = 0u;
    no_load_status.distance_m = 0.0f;
    no_load_status.brake_distance_m = 0.0f;
    no_load_start_ms = now_ms;
    no_load_previous_left_count = encoder_get_left_total_count();
    no_load_previous_right_count = encoder_get_right_total_count();
    no_load_marker_active = 0u;
    no_load_line_loss_active = 0u;
    no_load_sensor_offline_active = 0u;
    no_load_marker_distance_m = 0.0f;
    motor_app_set_base_rpm(NO_LOAD_LAP_CRUISE_RPM);
    motor_app_set_line_follow_enabled(1u);
    heartbeat_hw_uart_send_string("[no-load] lap start\r\n");
    return 1u;
}

void no_load_lap_app_process(void)
{
    const line_control_output_t *line;
    uint32 now_ms;

    if (NO_LOAD_LAP_RUNNING != no_load_status.state)
    {
        return;
    }

    now_ms = heartbeat_get_ms();
    no_load_status.elapsed_ms = now_ms - no_load_start_ms;
    no_load_update_distance();

    if (MOTOR_APP_MODE_LINE_FOLLOW != motor_app_get_mode())
    {
        no_load_finish(NO_LOAD_LAP_CHASSIS_STOPPED, now_ms);
        return;
    }
    if (0u == grayscale_is_online())
    {
        if (0u == no_load_sensor_offline_active)
        {
            no_load_sensor_offline_active = 1u;
            no_load_sensor_offline_start_ms = now_ms;
        }
        else if ((now_ms - no_load_sensor_offline_start_ms) >=
                 NO_LOAD_LAP_SENSOR_OFFLINE_TIMEOUT_MS)
        {
            no_load_finish(NO_LOAD_LAP_SENSOR_OFFLINE, now_ms);
            return;
        }
    }
    else
    {
        no_load_sensor_offline_active = 0u;
    }
    if (no_load_status.elapsed_ms >= NO_LOAD_LAP_TIMEOUT_MS)
    {
        no_load_finish(NO_LOAD_LAP_TIMEOUT, now_ms);
        return;
    }

    line = line_control_get_output();
    if (0u != line->line_lost)
    {
        if (0u == no_load_line_loss_active)
        {
            no_load_line_loss_active = 1u;
            no_load_line_loss_start_ms = now_ms;
        }
        else if ((now_ms - no_load_line_loss_start_ms) >=
                 NO_LOAD_LAP_LINE_LOSS_TIMEOUT_MS)
        {
            no_load_finish(NO_LOAD_LAP_LINE_LOST, now_ms);
            return;
        }
    }
    else
    {
        no_load_line_loss_active = 0u;
    }

    if ((0u == no_load_status.approach_active) &&
        (no_load_status.distance_m >= NO_LOAD_LAP_APPROACH_DISTANCE_M))
    {
        no_load_status.approach_active = 1u;
        motor_app_set_base_rpm(NO_LOAD_LAP_APPROACH_RPM);
        heartbeat_hw_uart_send_string("[no-load] approach A\r\n");
    }
    if (no_load_status.distance_m >= NO_LOAD_LAP_ARM_DISTANCE_M)
    {
        no_load_status.finish_armed = 1u;
    }

    if (0u != no_load_status.brake_active)
    {
        no_load_status.brake_distance_m =
            no_load_status.distance_m - no_load_marker_distance_m;
        if (no_load_status.brake_distance_m >=
            NO_LOAD_LAP_POST_MARKER_DISTANCE_M)
        {
            no_load_finish(NO_LOAD_LAP_COMPLETE, now_ms);
        }
        return;
    }

    if ((0u != no_load_status.finish_armed) &&
        (0u != line->marker_detected))
    {
        if (0u == no_load_marker_active)
        {
            no_load_marker_active = 1u;
            no_load_marker_start_ms = now_ms;
            no_load_marker_distance_m = no_load_status.distance_m;
        }
        else if ((now_ms - no_load_marker_start_ms) >=
                 NO_LOAD_LAP_MARKER_DEBOUNCE_MS)
        {
            no_load_status.brake_active = 1u;
            no_load_status.approach_active = 1u;
            no_load_status.brake_distance_m =
                no_load_status.distance_m - no_load_marker_distance_m;
            motor_app_set_base_rpm(NO_LOAD_LAP_APPROACH_RPM);
            heartbeat_hw_uart_send_string("[no-load] marker; braking 230mm\r\n");
        }
    }
    else
    {
        no_load_marker_active = 0u;
    }
}

void no_load_lap_app_stop(void)
{
    if (NO_LOAD_LAP_RUNNING == no_load_status.state)
    {
        no_load_finish(NO_LOAD_LAP_USER_STOP, heartbeat_get_ms());
    }
}

uint8 no_load_lap_app_is_running(void)
{
    return (NO_LOAD_LAP_RUNNING == no_load_status.state) ? 1u : 0u;
}

const no_load_lap_status_t *no_load_lap_app_get_status(void)
{
    return &no_load_status;
}
