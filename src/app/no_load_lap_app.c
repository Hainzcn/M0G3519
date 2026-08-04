#include "no_load_lap_app.h"

#include <stdio.h>

#include "control_config.h"
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
#include "balance_simple_app.h"
#endif
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
static uint32 no_load_line_loss_start_ms;
static uint32 no_load_sensor_offline_start_ms;
static uint8 no_load_line_loss_active;
static uint8 no_load_sensor_offline_active;
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
static uint8 no_load_restore_balance;
#endif

static uint8 no_load_is_active(void)
{
    return ((NO_LOAD_LAP_RUNNING == no_load_status.state) ||
            (NO_LOAD_LAP_POST_MARKER == no_load_status.state)) ? 1u : 0u;
}

static uint8 no_load_read_finish_marker(uint8 *sensor_mask,
                                        uint8 *active_count)
{
    const uint8 *values = grayscale_get_values();
    uint8 index;
    uint8 mask = 0u;
    uint8 count = 0u;

    for (index = 0u; index < GRAYSCALE_CHANNELS; index++)
    {
        if (LINE_BLACK_ACTIVE_LEVEL == values[index])
        {
            mask |= (uint8)(1u << index);
            count++;
        }
    }
    *sensor_mask = mask;
    *active_count = count;
    return (count >=
            NO_LOAD_LAP_MARKER_MIN_ACTIVE_COUNT) ? 1u : 0u;
}

static float no_load_post_marker_rpm(float distance_m)
{
    float remaining_m = NO_LOAD_LAP_POST_MARKER_DISTANCE_M - distance_m;
    float remaining_ratio;

    if (remaining_m <= 0.0f)
    {
        return 0.0f;
    }
    remaining_ratio = remaining_m /
        NO_LOAD_LAP_POST_MARKER_DISTANCE_M;
    if (remaining_ratio > 1.0f)
    {
        remaining_ratio = 1.0f;
    }
    remaining_ratio *= remaining_ratio;
    return NO_LOAD_LAP_POST_MARKER_MIN_RPM +
        (NO_LOAD_LAP_CRUISE_RPM - NO_LOAD_LAP_POST_MARKER_MIN_RPM) *
        remaining_ratio;
}

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
    char message[144];
    uint32 distance_mm = (uint32)(no_load_status.distance_m * 1000.0f);
    uint32 marker_mm =
        (uint32)(no_load_status.marker_distance_m * 1000.0f);
    uint32 brake_mm =
        (uint32)(no_load_status.brake_distance_m * 1000.0f);

    snprintf(message, sizeof(message),
        "[no-load] end=%u,t=%lu,total=%lu,marker=%lu,post=%lu\r\n",
        (unsigned int)no_load_status.state,
        (unsigned long)no_load_status.elapsed_ms,
        (unsigned long)distance_mm,
        (unsigned long)marker_mm,
        (unsigned long)brake_mm);
    heartbeat_hw_uart_send_string(message);
}

static void no_load_finish(no_load_lap_state_enum state, uint32 now_ms)
{
    if (NO_LOAD_LAP_COMPLETE == state)
    {
        motor_app_brake();
    }
    else
    {
        motor_app_stop();
    }
    no_load_status.elapsed_ms = now_ms - no_load_start_ms;
    no_load_status.state = state;
    no_load_line_loss_active = 0u;
    no_load_sensor_offline_active = 0u;
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    if (0u != no_load_restore_balance)
    {
        no_load_restore_balance = 0u;
        if (0u == balance_simple_app_start())
        {
            heartbeat_hw_uart_send_string(
                "[no-load] balance restart rejected\r\n");
        }
    }
#endif
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
    no_load_status.marker_distance_m = 0.0f;
    no_load_status.brake_distance_m = 0.0f;
    no_load_start_ms = heartbeat_get_ms();
    no_load_previous_left_count = encoder_get_left_total_count();
    no_load_previous_right_count = encoder_get_right_total_count();
    no_load_line_loss_start_ms = 0u;
    no_load_sensor_offline_start_ms = 0u;
    no_load_line_loss_active = 0u;
    no_load_sensor_offline_active = 0u;
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    no_load_restore_balance = 0u;
#endif
}

uint8 no_load_lap_app_start(void)
{
    uint32 now_ms = heartbeat_get_ms();

    if (MOTOR_APP_MODE_DISABLED != motor_app_get_mode())
    {
        return 0u;
    }

#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    balance_simple_app_disable();
    no_load_restore_balance = 1u;
#endif

    no_load_status.state = NO_LOAD_LAP_RUNNING;
    no_load_status.finish_armed = 0u;
    no_load_status.approach_active = 0u;
    no_load_status.brake_active = 0u;
    no_load_status.elapsed_ms = 0u;
    no_load_status.distance_m = 0.0f;
    no_load_status.marker_distance_m = 0.0f;
    no_load_status.brake_distance_m = 0.0f;
    no_load_start_ms = now_ms;
    no_load_previous_left_count = encoder_get_left_total_count();
    no_load_previous_right_count = encoder_get_right_total_count();
    no_load_line_loss_active = 0u;
    no_load_sensor_offline_active = 0u;
    motor_app_set_base_rpm(NO_LOAD_LAP_CRUISE_RPM);
    motor_app_set_line_follow_enabled(1u);
    heartbeat_hw_uart_send_string("[no-load] lap start\r\n");
    return 1u;
}

void no_load_lap_app_process(void)
{
    const line_control_output_t *line;
    uint32 now_ms;
    uint8 sensor_online;
    uint8 marker_sensor_mask = 0u;
    uint8 marker_active_count = 0u;

    if (0u == no_load_is_active())
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
    sensor_online = grayscale_is_online();
    if (0u == sensor_online)
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
    if ((NO_LOAD_LAP_RUNNING == no_load_status.state) &&
        (no_load_status.elapsed_ms >= NO_LOAD_LAP_TIMEOUT_MS))
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

    /* Ignore start-area graphics and letter strokes until a full lap is plausible. */
    if (no_load_status.distance_m > NO_LOAD_LAP_MARKER_MIN_DISTANCE_M)
    {
        no_load_status.finish_armed = 1u;
    }

    if (NO_LOAD_LAP_POST_MARKER == no_load_status.state)
    {
        float target_rpm;

        no_load_status.brake_distance_m =
            no_load_status.distance_m -
            no_load_status.marker_distance_m;
        if (no_load_status.brake_distance_m >=
            NO_LOAD_LAP_POST_MARKER_DISTANCE_M)
        {
            no_load_finish(NO_LOAD_LAP_COMPLETE, now_ms);
            return;
        }
        target_rpm = no_load_post_marker_rpm(
            no_load_status.brake_distance_m);
        motor_app_set_base_rpm_immediate(target_rpm);
        return;
    }

    if ((0u != no_load_status.finish_armed) &&
        (0u != sensor_online) &&
        (0u != no_load_read_finish_marker(&marker_sensor_mask,
                                           &marker_active_count)))
    {
        char message[112];

        no_load_status.state = NO_LOAD_LAP_POST_MARKER;
        no_load_status.approach_active = 1u;
        no_load_status.brake_active = 1u;
        no_load_status.marker_distance_m = no_load_status.distance_m;
        no_load_status.brake_distance_m = 0.0f;
        motor_app_set_rapid_brake_enabled(1u);
        motor_app_set_base_rpm_immediate(NO_LOAD_LAP_CRUISE_RPM);
        snprintf(message, sizeof(message),
            "[no-load] marker latched,total=%lu,mask=%02X,n=%u; post=210mm\r\n",
            (unsigned long)(no_load_status.distance_m * 1000.0f),
            (unsigned int)marker_sensor_mask,
            (unsigned int)marker_active_count);
        heartbeat_hw_uart_send_string(message);
    }
}

void no_load_lap_app_stop(void)
{
    if (0u != no_load_is_active())
    {
        no_load_finish(NO_LOAD_LAP_USER_STOP, heartbeat_get_ms());
    }
}

uint8 no_load_lap_app_is_running(void)
{
    return no_load_is_active();
}

const no_load_lap_status_t *no_load_lap_app_get_status(void)
{
    return &no_load_status;
}
