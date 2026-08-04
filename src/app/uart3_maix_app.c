#include "uart3_maix_app.h"

#include <stdio.h>

#include "control_config.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "uart3_maix_hw.h"
#include "vision_link.h"

#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
#include "imu.h"
#endif
#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"
#endif
#if (BALANCE_CONTROL_ENABLE != 0u)
#include "balance_app.h"
#endif
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
#include "balance_simple_app.h"
#endif
#if ((UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG) && \
     (EMM42_BALANCE_DEMO_ENABLE != 0u))
#include "balance_linkage.h"
#include "emm42_demo_app.h"
#endif

#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
static void balance_simple_send_diagnostic(void)
{
    char message[256];
    const balance_simple_status_t *status =
        balance_simple_app_get_status();
    uint32 vision_age = (0xFFFFFFFFu == status->vision_age_ms) ?
        999999u : status->vision_age_ms;
    uint32 motor_age = (0xFFFFFFFFu == status->motor_position_age_ms) ?
        999999u : status->motor_position_age_ms;

    snprintf(message, sizeof(message),
        "[balance-simple] ctl=angle-pi-bias-v5 fb=50 st=%u fault=%u fl=%04X sat=%04X "
        "seq=%u age=%lu x=%d xh=%d v=%d vr=%d tr=%d th=%d "
        "the=%d om=%d rpm=%d mp=%d mage=%lu err=%u\r\n",
        (unsigned int)status->state,
        (unsigned int)status->fault,
        (unsigned int)status->flags,
        (unsigned int)status->saturation_flags,
        (unsigned int)status->vision_sequence,
        (unsigned long)vision_age,
        (int)(status->raw_position_m * 10000.0f),
        (int)(status->estimated_position_m * 10000.0f),
        (int)(status->estimated_velocity_mps * 1000.0f),
        (int)(status->target_velocity_mps * 1000.0f),
        (int)(status->target_beam_angle_deg * 100.0f),
        (int)(status->measured_beam_angle_deg * 100.0f),
        (int)(status->beam_angle_error_deg * 100.0f),
        (int)(status->omega_command_deg_s * 100.0f),
        (int)status->motor_rpm_command,
        (int)(status->motor_position_deg * 100.0f),
        (unsigned long)motor_age,
        (unsigned int)status->command_error_count);
    heartbeat_hw_uart_send_string(message);
}
#endif

#define UART3_MAIX_DIAGNOSTIC_PERIOD_MS       (1000u)

static uint32 uart3_maix_last_diagnostic_ms;

static void vision_link_send_diagnostic(void)
{
    char message[256];
    vision_link_status_t status;

    vision_link_get_status(&status);
    snprintf(message, sizeof(message),
        "[vision] on=%u valid=%u age=%u/%u seq=%u boot=%04X "
        "ok=%u accept=%u crc=%u hdr=%u sem=%u dup=%u back=%u gap=%u "
        "restart=%u ovf=%u\r\n",
        (unsigned int)status.link_online,
        (unsigned int)status.measurement_valid,
        (unsigned int)status.link_age_ms,
        (unsigned int)status.measurement_age_ms,
        (unsigned int)status.last_sequence,
        (unsigned int)status.boot_id,
        (unsigned int)status.crc_ok_frames,
        (unsigned int)status.accepted_frames,
        (unsigned int)status.crc_errors,
        (unsigned int)status.header_errors,
        (unsigned int)status.semantic_errors,
        (unsigned int)status.duplicate_frames,
        (unsigned int)status.backward_frames,
        (unsigned int)status.sequence_gap_frames,
        (unsigned int)status.boot_changes,
        (unsigned int)status.uart_rx_overflows);
    heartbeat_hw_uart_send_string(message);
}

#if (BALANCE_CONTROL_ENABLE != 0u)
static void balance_send_diagnostic(void)
{
    char message[256];
    const balance_app_status_t *status = balance_app_get_status();
    uint32 age = (0xFFFFFFFFu == status->vision_age_ms) ? 999999u :
        status->vision_age_ms;

    snprintf(message, sizeof(message),
        "[balance] st=%u fault=%u fl=%02X ctl=%02X raw=%02X "
        "conf=%u/%u seq=%u/%u age=%u pos=%d vel=%d "
        "tgt=%d ref=%d/%d ph=%u seqst=%u lever=%d/%d "
        "mt=%d mf=%d err=%u drop=%u\r\n",
        (unsigned int)status->state,
        (unsigned int)status->fault,
        (unsigned int)status->flags,
        (unsigned int)status->control_flags,
        (unsigned int)status->vision_raw_flags,
        (unsigned int)status->vision_confidence,
        (unsigned int)status->vision_raw_confidence,
        (unsigned int)status->vision_sequence,
        (unsigned int)status->vision_raw_sequence,
        (unsigned int)age,
        (int)status->vision_raw_position_dmm,
        (int)status->vision_raw_velocity_mm_s,
        (int)(status->target_position_m * 10000.0f),
        (int)(status->reference_position_m * 10000.0f),
        (int)(status->reference_velocity_mps * 1000.0f),
        (unsigned int)status->motion_phase,
        (unsigned int)status->sequence_state,
        (int)(status->lever_angle_deg * 100.0f),
        (int)(status->actual_lever_angle_deg * 100.0f),
        (int)(status->motor_target_deg * 100.0f),
        (int)(status->motor_feedback_deg * 100.0f),
        (unsigned int)status->command_error_count,
        (unsigned int)heartbeat_hw_uart_get_drop_count());
    heartbeat_hw_uart_send_string(message);
}
#endif

#if ((UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG) || \
     ((UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG) && \
      (BALL_RETURN_DEMO_ENABLE == 0u)))
static void telemetry_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFu);
    buffer[1] = (uint8)(value >> 8u);
}

static void telemetry_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFu);
    buffer[1] = (uint8)((value >> 8u) & 0xFFu);
    buffer[2] = (uint8)((value >> 16u) & 0xFFu);
    buffer[3] = (uint8)((value >> 24u) & 0xFFu);
}

static int16 telemetry_float_to_i16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }
    return (int16)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void telemetry_write_scaled_i16(
    uint8 *buffer, float value, float scale)
{
    telemetry_write_u16_le(buffer,
        (uint16)telemetry_float_to_i16(value, scale));
}

static uint16 telemetry_crc16_ccitt_false(
    const uint8 *data, uint16 length)
{
    uint16 crc = 0xFFFFu;
    uint16 index;
    uint8 bit;

    for (index = 0u; index < length; index++)
    {
        crc ^= (uint16)data[index] << 8u;
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (0u != (crc & 0x8000u)) ?
                (uint16)((crc << 1u) ^ 0x1021u) : (uint16)(crc << 1u);
        }
    }
    return crc;
}
#endif

#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
#define CHASSIS_TELEMETRY_PERIOD_MS           (10u)
#define CHASSIS_TELEMETRY_FRAME_SIZE          (56u)
#define CHASSIS_FLAG_CONTROL_ACTIVE           (0x01u)
#define CHASSIS_FLAG_LINE_VALID               (0x02u)
#define CHASSIS_FLAG_LINE_LOST                (0x04u)
#define CHASSIS_FLAG_MARKER                   (0x08u)
#define CHASSIS_FLAG_LEFT_SATURATED           (0x10u)
#define CHASSIS_FLAG_RIGHT_SATURATED          (0x20u)
#define CHASSIS_FLAG_KINEMATICS_VALID         (0x40u)
#define CHASSIS_FLAG_UART_RX_OVERFLOW         (0x80u)

static uint32 chassis_telemetry_last_ms;
static uint16 chassis_telemetry_sequence;

static uint8 chassis_telemetry_flags(
    motor_app_mode_enum mode,
    const line_control_output_t *line,
    const wheel_speed_control_status_t *wheel)
{
    uint8 flags = 0u;

    if (MOTOR_APP_MODE_DISABLED != mode) flags |= CHASSIS_FLAG_CONTROL_ACTIVE;
    if (0u != line->line_valid) flags |= CHASSIS_FLAG_LINE_VALID;
    if (0u != line->line_lost) flags |= CHASSIS_FLAG_LINE_LOST;
    if (0u != line->marker_detected) flags |= CHASSIS_FLAG_MARKER;
    if (0u != wheel->left_saturated) flags |= CHASSIS_FLAG_LEFT_SATURATED;
    if (0u != wheel->right_saturated) flags |= CHASSIS_FLAG_RIGHT_SATURATED;
    if (0u != wheel->kinematics_valid) flags |= CHASSIS_FLAG_KINEMATICS_VALID;
    if (0u != uart3_maix_hw_get_rx_overflow_count())
    {
        flags |= CHASSIS_FLAG_UART_RX_OVERFLOW;
    }
    return flags;
}

static void chassis_telemetry_send(uint32 now_ms)
{
    uint8 frame[CHASSIS_TELEMETRY_FRAME_SIZE];
    const line_control_output_t *line = line_control_get_output();
    const wheel_speed_control_status_t *wheel =
        wheel_speed_control_get_status();
    imu_snapshot_t imu;
    motor_app_mode_enum mode = motor_app_get_mode();
    uint32 rx_overflow = uart3_maix_hw_get_rx_overflow_count();
    uint16 crc;

    imu_get_snapshot(&imu);
    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = 0x02u;
    frame[3] = 0x81u;
    frame[4] = CHASSIS_TELEMETRY_FRAME_SIZE;
    frame[5] = (uint8)mode;
    frame[6] = chassis_telemetry_flags(mode, line, wheel);
    frame[7] = 0u;
    telemetry_write_u16_le(&frame[8], chassis_telemetry_sequence);
    telemetry_write_u32_le(&frame[10], now_ms);
    telemetry_write_scaled_i16(&frame[14], wheel->planned_speed_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[16], wheel->planned_accel_mps2, 1000.0f);
    telemetry_write_scaled_i16(&frame[18], wheel->left_target_rpm, 10.0f);
    telemetry_write_scaled_i16(&frame[20], wheel->right_target_rpm, 10.0f);
    telemetry_write_scaled_i16(&frame[22], wheel->left_measured_rpm, 10.0f);
    telemetry_write_scaled_i16(&frame[24], wheel->right_measured_rpm, 10.0f);
    telemetry_write_scaled_i16(&frame[26], wheel->left_feedforward_pwm, 1.0f);
    telemetry_write_scaled_i16(&frame[28], wheel->right_feedforward_pwm, 1.0f);
    telemetry_write_scaled_i16(&frame[30], wheel->left_feedback_pwm, 1.0f);
    telemetry_write_scaled_i16(&frame[32], wheel->right_feedback_pwm, 1.0f);
    telemetry_write_scaled_i16(&frame[34], (float)wheel->left_duty, 1.0f);
    telemetry_write_scaled_i16(&frame[36], (float)wheel->right_duty, 1.0f);
    telemetry_write_scaled_i16(&frame[38], wheel->measured_speed_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[40], wheel->measured_accel_mps2, 1000.0f);
    telemetry_write_scaled_i16(&frame[42], line->error, 1.0f);
    telemetry_write_u32_le(&frame[44], uart3_maix_hw_get_tx_drop_count());
    telemetry_write_u16_le(&frame[48],
        (uint16)((rx_overflow > 65535u) ? 65535u : rx_overflow));
    if (0u != (imu.flags & IMU_FLAG_ACCEL))
    {
        telemetry_write_scaled_i16(&frame[50], imu.accel.ax, 1000.0f);
    }
    else
    {
        telemetry_write_u16_le(&frame[50], 0x8000u);
    }
    frame[52] = imu.flags;
    frame[53] = 0u;
    crc = telemetry_crc16_ccitt_false(frame, 54u);
    telemetry_write_u16_le(&frame[54], crc);
    (void)uart3_maix_hw_write_atomic(frame, CHASSIS_TELEMETRY_FRAME_SIZE);
    chassis_telemetry_sequence++;
}
#endif

#if (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)
#define BALANCE_TELEMETRY_PERIOD_MS           (10u)

static uint32 balance_telemetry_last_ms;
static uint16 balance_telemetry_sequence;

#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
#define BALANCE_DEMO_TELEMETRY_FRAME_SIZE     (40u)
#define BALANCE_DEMO_FLAG_ACTIVE              (0x01u)
#define BALANCE_DEMO_FLAG_LEVER_VALID         (0x02u)
#define BALANCE_DEMO_FLAG_MOTOR_VALID         (0x04u)
#define BALANCE_DEMO_FLAG_VISION_ONLINE       (0x08u)
#define BALANCE_DEMO_FLAG_VISION_MEASURED     (0x10u)
#define BALANCE_DEMO_FLAG_RECORDING           (0x20u)
#define BALANCE_DEMO_FLAG_ERROR               (0x40u)

static void balance_telemetry_send(uint32 now_ms)
{
    uint8 frame[BALANCE_DEMO_TELEMETRY_FRAME_SIZE];
    vision_link_snapshot_t vision;
    vision_link_status_t vision_status;
    uint8 flags = 0u;
    uint8 has_vision;
    float lever_feedback_deg = 0.0f;
    uint16 vision_age;
    uint16 crc;

    vision_link_get_status(&vision_status);
    has_vision = vision_link_get_latest_snapshot(&vision);
    if (0u != emm42_demo_app_is_active())
    {
        flags |= BALANCE_DEMO_FLAG_ACTIVE;
    }
    if (0u != emm42_demo_app_is_motor_feedback_valid())
    {
        flags |= BALANCE_DEMO_FLAG_MOTOR_VALID;
        if (0u != balance_linkage_physical_lever_from_motor_deg(
                emm42_demo_app_get_motor_feedback_deg(),
                &lever_feedback_deg))
        {
            flags |= BALANCE_DEMO_FLAG_LEVER_VALID;
        }
    }
    if (0u != vision_status.link_online)
    {
        flags |= BALANCE_DEMO_FLAG_VISION_ONLINE;
    }
    if ((0u != has_vision) &&
        (0u != (vision.flags & VISION_LINK_FLAG_MEASURED_VALID)))
    {
        flags |= BALANCE_DEMO_FLAG_VISION_MEASURED;
    }
    if (0u != emm42_demo_app_is_recording())
    {
        flags |= BALANCE_DEMO_FLAG_RECORDING;
    }
    if (EMM42_DEMO_ERROR == emm42_demo_app_get_state())
    {
        flags |= BALANCE_DEMO_FLAG_ERROR;
    }

    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = 0x05u;
    frame[3] = 0x82u;
    frame[4] = BALANCE_DEMO_TELEMETRY_FRAME_SIZE;
    frame[5] = flags;
    frame[6] = (uint8)emm42_demo_app_get_state();
    frame[7] = 0u;
    telemetry_write_u16_le(&frame[8], balance_telemetry_sequence);
    telemetry_write_u32_le(&frame[10], now_ms);
    telemetry_write_u16_le(&frame[14], emm42_demo_app_get_trial_id());
    telemetry_write_scaled_i16(&frame[16],
        emm42_demo_app_get_target_lever_deg(), 100.0f);
    telemetry_write_scaled_i16(&frame[18],
        emm42_demo_app_get_target_motor_deg(), 100.0f);
    if (0u != (flags & BALANCE_DEMO_FLAG_MOTOR_VALID))
    {
        telemetry_write_scaled_i16(&frame[20],
            emm42_demo_app_get_motor_feedback_deg(), 100.0f);
    }
    else
    {
        telemetry_write_u16_le(&frame[20], 0x8000u);
    }
    if (0u != (flags & BALANCE_DEMO_FLAG_LEVER_VALID))
    {
        telemetry_write_scaled_i16(&frame[22], lever_feedback_deg, 100.0f);
    }
    else
    {
        telemetry_write_u16_le(&frame[22], 0x8000u);
    }
    if (0u != has_vision)
    {
        telemetry_write_u16_le(&frame[24], vision.sequence);
        telemetry_write_u32_le(&frame[26], vision.capture_ms);
        telemetry_write_u16_le(&frame[30], (uint16)vision.position_dmm);
        telemetry_write_u16_le(&frame[32], (uint16)vision.velocity_mm_s);
        frame[34] = vision.confidence;
        frame[35] = vision.flags;
        vision_age = (0xFFFFFFFFu == vision_status.measurement_age_ms) ?
            0xFFFFu :
            (uint16)((vision_status.measurement_age_ms > 65534u) ?
                     65534u : vision_status.measurement_age_ms);
        telemetry_write_u16_le(&frame[36], vision_age);
    }
    else
    {
        telemetry_write_u16_le(&frame[24], 0u);
        telemetry_write_u32_le(&frame[26], 0u);
        telemetry_write_u16_le(&frame[30], 0x8000u);
        telemetry_write_u16_le(&frame[32], 0x8000u);
        frame[34] = 0u;
        frame[35] = 0u;
        telemetry_write_u16_le(&frame[36], 0xFFFFu);
    }
    crc = telemetry_crc16_ccitt_false(frame, 38u);
    telemetry_write_u16_le(&frame[38], crc);
    (void)uart3_maix_hw_write_atomic(
        frame, BALANCE_DEMO_TELEMETRY_FRAME_SIZE);
    balance_telemetry_sequence++;
}
#elif (BALL_RETURN_DEMO_ENABLE != 0u)
static void balance_telemetry_send(uint32 now_ms)
{
    /* This open-loop demo has no vision-derived state to report on UART3. */
    (void)now_ms;
}
#elif (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
#define BALANCE_TELEMETRY_FRAME_SIZE          (68u)

static void balance_telemetry_send(uint32 now_ms)
{
    uint8 frame[BALANCE_TELEMETRY_FRAME_SIZE];
    const balance_simple_status_t *status =
        balance_simple_app_get_status();
    uint16 vision_age;
    uint16 motor_age;
    uint16 motor_velocity_age;
    uint16 crc;

    vision_age = (0xFFFFFFFFu == status->vision_age_ms) ? 0xFFFFu :
        (uint16)((status->vision_age_ms > 65534u) ?
                 65534u : status->vision_age_ms);
    motor_age = (0xFFFFFFFFu == status->motor_position_age_ms) ? 0xFFFFu :
        (uint16)((status->motor_position_age_ms > 65534u) ?
                 65534u : status->motor_position_age_ms);
    motor_velocity_age =
        (0xFFFFFFFFu == status->motor_velocity_age_ms) ? 0xFFFFu :
        (uint16)((status->motor_velocity_age_ms > 65534u) ?
                 65534u : status->motor_velocity_age_ms);
    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = 0x07u;
    frame[3] = 0x82u;
    frame[4] = BALANCE_TELEMETRY_FRAME_SIZE;
    frame[5] = (uint8)status->flags;
    frame[6] = (uint8)status->state;
    frame[7] = (uint8)status->fault;
    telemetry_write_u16_le(&frame[8], balance_telemetry_sequence);
    telemetry_write_u32_le(&frame[10], now_ms);
    telemetry_write_u16_le(&frame[14], status->vision_sequence);
    telemetry_write_u32_le(&frame[16], status->capture_ms);
    telemetry_write_u16_le(&frame[20], vision_age);
    telemetry_write_scaled_i16(&frame[22], status->raw_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[24],
        status->estimated_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[26],
        status->estimated_velocity_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[28],
        status->target_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[30],
        status->position_error_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[32],
        status->velocity_limit_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[34],
        status->target_velocity_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[36],
        status->effective_kv_deg_per_mm, 1000.0f);
    telemetry_write_scaled_i16(&frame[38],
        status->omega_command_deg_s, 100.0f);
    telemetry_write_scaled_i16(&frame[40],
        status->motor_rpm_requested, 100.0f);
    telemetry_write_u16_le(&frame[42],
        (uint16)status->motor_rpm_command);
    telemetry_write_scaled_i16(&frame[44],
        status->motor_position_deg, 100.0f);
    telemetry_write_u16_le(&frame[46], motor_age);
    telemetry_write_u16_le(&frame[48], status->saturation_flags);
    telemetry_write_u16_le(&frame[50], status->command_error_count);
    telemetry_write_u16_le(&frame[52], status->emm42_rx_overflow_count);
    frame[54] = status->vision_confidence;
    frame[55] = status->vision_flags;
    telemetry_write_scaled_i16(&frame[56],
        status->integral_velocity_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[58],
        status->filtered_ball_accel_mps2, 1000.0f);
    telemetry_write_u16_le(&frame[60], status->flags);
    if (0u != (status->flags &
               BALANCE_SIMPLE_FLAG_MOTOR_VELOCITY_VALID))
    {
        telemetry_write_u16_le(&frame[62],
                               (uint16)status->motor_rpm_actual);
    }
    else
    {
        telemetry_write_u16_le(&frame[62], 0x8000u);
    }
    telemetry_write_u16_le(&frame[64], motor_velocity_age);
    crc = telemetry_crc16_ccitt_false(frame, 66u);
    telemetry_write_u16_le(&frame[66], crc);
    (void)uart3_maix_hw_write_atomic(frame, BALANCE_TELEMETRY_FRAME_SIZE);
    balance_telemetry_sequence++;
}
#else
#define BALANCE_TELEMETRY_FRAME_SIZE          (64u)

static void balance_telemetry_send(uint32 now_ms)
{
    uint8 frame[BALANCE_TELEMETRY_FRAME_SIZE];
    const balance_app_status_t *status = balance_app_get_status();
    uint16 vision_age;
    uint16 crc;

    vision_age = (0xFFFFFFFFu == status->vision_age_ms) ? 0xFFFFu :
        (uint16)((status->vision_age_ms > 65534u) ?
                 65534u : status->vision_age_ms);
    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = 0x04u;
    frame[3] = 0x82u;
    frame[4] = BALANCE_TELEMETRY_FRAME_SIZE;
    frame[5] = status->flags;
    frame[6] = (uint8)status->state;
    frame[7] = (uint8)status->fault;
    telemetry_write_u16_le(&frame[8], balance_telemetry_sequence);
    telemetry_write_u32_le(&frame[10], now_ms);
    telemetry_write_u16_le(&frame[14], status->vision_sequence);
    telemetry_write_scaled_i16(&frame[16],
        status->estimated_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[18],
        status->estimated_velocity_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[20],
        status->predicted_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[22],
        status->predicted_velocity_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[24],
        status->reference_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[26],
        status->reference_velocity_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[28],
        status->target_position_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[30],
        status->feedforward_accel_mps2, 1000.0f);
    telemetry_write_scaled_i16(&frame[32],
        status->feedback_accel_mps2, 1000.0f);
    telemetry_write_scaled_i16(&frame[34],
        status->desired_ball_accel_mps2, 1000.0f);
    telemetry_write_scaled_i16(&frame[36],
        status->velocity_limit_mps, 1000.0f);
    telemetry_write_scaled_i16(&frame[38],
        status->brake_distance_m, 10000.0f);
    telemetry_write_scaled_i16(&frame[40],
        status->raw_lever_angle_deg, 100.0f);
    telemetry_write_scaled_i16(&frame[42],
        status->lever_angle_deg, 100.0f);
    if (0u != (status->flags & BALANCE_APP_FLAG_LEVER_FEEDBACK_VALID))
    {
        telemetry_write_scaled_i16(&frame[44],
            status->actual_lever_angle_deg, 100.0f);
    }
    else
    {
        telemetry_write_u16_le(&frame[44], 0x8000u);
    }
    telemetry_write_scaled_i16(&frame[46],
        status->motor_target_deg, 100.0f);
    if (0u != (status->flags & BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID))
    {
        telemetry_write_scaled_i16(&frame[48],
            status->motor_feedback_deg, 100.0f);
    }
    else
    {
        telemetry_write_u16_le(&frame[48], 0x8000u);
    }
    telemetry_write_u16_le(&frame[50], vision_age);
    telemetry_write_u16_le(&frame[52], status->control_flags);
    frame[54] = (uint8)status->control_phase;
    frame[55] = (uint8)status->friction_mode;
    frame[56] = status->vision_confidence;
    frame[57] = 0u;
    telemetry_write_u16_le(&frame[58], status->command_error_count);
    telemetry_write_u16_le(&frame[60], status->emm42_rx_overflow_count);
    crc = telemetry_crc16_ccitt_false(frame, 62u);
    telemetry_write_u16_le(&frame[62], crc);
    (void)uart3_maix_hw_write_atomic(frame, BALANCE_TELEMETRY_FRAME_SIZE);
    balance_telemetry_sequence++;
}
#endif
#endif

void uart3_maix_app_init(void)
{
    uint32 now_ms;

    uart3_maix_hw_init();
    vision_link_init();
    now_ms = heartbeat_get_ms();
    uart3_maix_last_diagnostic_ms = now_ms;
#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
    chassis_telemetry_last_ms = now_ms;
    chassis_telemetry_sequence = 0u;
    heartbeat_hw_uart_send_string(
        "[mode] uart3 chassis telemetry debug, 0x81/100Hz\r\n");
#elif (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)
    balance_telemetry_last_ms = now_ms;
    balance_telemetry_sequence = 0u;
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
    heartbeat_hw_uart_send_string(
        "[mode] uart3 ball dynamics calibration V5, 0x82/100Hz\r\n");
#elif (BALL_RETURN_DEMO_ENABLE != 0u)
    heartbeat_hw_uart_send_string(
        "[mode] uart3 ball return demo, telemetry disabled\r\n");
#elif (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    heartbeat_hw_uart_send_string(
        "[mode] uart3 balance simple telemetry V6, 0x82/100Hz\r\n");
#else
    heartbeat_hw_uart_send_string(
        "[mode] uart3 balance control telemetry V4, 0x82/100Hz\r\n");
#endif
#else
    heartbeat_hw_uart_send_string(
        "[mode] uart3 normal, vision RX only, TX silent\r\n");
#endif
}

void uart3_maix_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
    uart3_maix_hw_tx_pump();
    if ((now_ms - chassis_telemetry_last_ms) >= CHASSIS_TELEMETRY_PERIOD_MS)
    {
        chassis_telemetry_last_ms = now_ms;
        chassis_telemetry_send(now_ms);
    }
    uart3_maix_hw_tx_pump();
#elif (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)
    uart3_maix_hw_tx_pump();
    if ((now_ms - balance_telemetry_last_ms) >= BALANCE_TELEMETRY_PERIOD_MS)
    {
        balance_telemetry_last_ms = now_ms;
        balance_telemetry_send(now_ms);
    }
    uart3_maix_hw_tx_pump();
#endif

    if ((now_ms - uart3_maix_last_diagnostic_ms) >=
        UART3_MAIX_DIAGNOSTIC_PERIOD_MS)
    {
        uart3_maix_last_diagnostic_ms = now_ms;
        vision_link_send_diagnostic();
#if (BALANCE_CONTROL_ENABLE != 0u)
        balance_send_diagnostic();
#endif
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
        balance_simple_send_diagnostic();
#endif
    }
}
