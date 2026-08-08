#include "uart3_maix_app.h"

#include <stdio.h>

#include "button.h"
#include "control_config.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "uart3_maix_hw.h"
#include "vision_link.h"
#include "wheel_speed_control.h"

#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"
#endif
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
#include "balance_simple_app.h"
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
        "[balance-simple] ctl=angle-pi-car-ff-v7 fb=50 st=%u fault=%u fl=%04X sat=%04X "
        "seq=%u age=%lu x=%d xh=%d v=%d vr=%d tr=%d th=%d "
        "the=%d om=%d rpm=%d mp=%d mage=%lu ca=%d cff=%d cfs=%u err=%u\r\n",
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
        (int)(status->car_accel_mps2 * 1000.0f),
        (int)(status->car_feedforward_angle_deg * 100.0f),
        (unsigned int)(status->car_feedforward_scale * 100.0f),
        (unsigned int)status->command_error_count);
    heartbeat_hw_uart_send_string(message);
}
#endif

#define UART3_MAIX_DIAGNOSTIC_PERIOD_MS       (1000u)

static uint32 uart3_maix_last_diagnostic_ms;

#define OPERATIONAL_TELEMETRY_PERIOD_MS       (10u)
#define OPERATIONAL_TELEMETRY_FRAME_SIZE      (24u)
#define OPERATIONAL_TELEMETRY_IMU_VALID       (0x01u)
#define OPERATIONAL_TELEMETRY_IMU_MAX_AGE_MS  (100u)

static uint32 operational_telemetry_last_ms;
static uint16 operational_telemetry_sequence;
static uint16 operational_button_sequence;
static button_id_t operational_button_previous;
static button_id_t operational_button_last_press;

static void operational_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFu);
    buffer[1] = (uint8)(value >> 8u);
}

static void operational_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFu);
    buffer[1] = (uint8)((value >> 8u) & 0xFFu);
    buffer[2] = (uint8)((value >> 16u) & 0xFFu);
    buffer[3] = (uint8)((value >> 24u) & 0xFFu);
}

static int16 operational_float_to_i16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static uint16 operational_crc16_ccitt_false(
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

static void operational_telemetry_update_button(void)
{
    button_id_t active = button_get_active();

    if ((BUTTON_ID_NONE != active) &&
        (active != operational_button_previous))
    {
        operational_button_last_press = active;
        operational_button_sequence++;
    }
    operational_button_previous = active;
}

static void operational_telemetry_send(uint32 now_ms)
{
    uint8 frame[OPERATIONAL_TELEMETRY_FRAME_SIZE];
    uint8 flags = 0u;
    imu_snapshot_t imu;
    const wheel_speed_control_status_t *wheel =
        wheel_speed_control_get_status();
    float forward_accel_mps2 = 0.0f;
    uint16 crc;

    imu_get_snapshot(&imu);
    if ((0u != (imu.flags & IMU_FLAG_ACCEL)) &&
        ((now_ms - imu.accel_time_ms) <=
         OPERATIONAL_TELEMETRY_IMU_MAX_AGE_MS))
    {
        flags |= OPERATIONAL_TELEMETRY_IMU_VALID;
        forward_accel_mps2 =
            imu.accel.ax * BALANCE_SIMPLE_CAR_ACCEL_SIGN;
    }

    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = 0x01u;
    frame[3] = 0x83u;
    frame[4] = OPERATIONAL_TELEMETRY_FRAME_SIZE;
    frame[5] = flags;
    frame[6] = (uint8)operational_button_last_press;
    frame[7] = (uint8)operational_button_previous;
    operational_write_u16_le(&frame[8], operational_button_sequence);
    operational_write_u16_le(&frame[10], operational_telemetry_sequence);
    operational_write_u32_le(&frame[12], now_ms);
    if (0u != (flags & OPERATIONAL_TELEMETRY_IMU_VALID))
    {
        operational_write_u16_le(&frame[16],
            (uint16)operational_float_to_i16(
                forward_accel_mps2, 1000.0f));
    }
    else
    {
        operational_write_u16_le(&frame[16], 0x8000u);
    }
    operational_write_u16_le(&frame[18],
        (uint16)operational_float_to_i16(
            wheel->left_measured_rpm, 10.0f));
    operational_write_u16_le(&frame[20],
        (uint16)operational_float_to_i16(
            wheel->right_measured_rpm, 10.0f));
    crc = operational_crc16_ccitt_false(frame, 22u);
    operational_write_u16_le(&frame[22], crc);
    (void)uart3_maix_hw_write_atomic(
        frame, OPERATIONAL_TELEMETRY_FRAME_SIZE);
    operational_telemetry_sequence++;
}

static void operational_telemetry_process(uint32 now_ms)
{
    operational_telemetry_update_button();
    uart3_maix_hw_tx_pump();
    if ((now_ms - operational_telemetry_last_ms) >=
        OPERATIONAL_TELEMETRY_PERIOD_MS)
    {
        operational_telemetry_last_ms += OPERATIONAL_TELEMETRY_PERIOD_MS;
        if ((now_ms - operational_telemetry_last_ms) >=
            OPERATIONAL_TELEMETRY_PERIOD_MS)
        {
            operational_telemetry_last_ms = now_ms;
        }
        operational_telemetry_send(now_ms);
    }
    uart3_maix_hw_tx_pump();
}

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


#if ((UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG) || \
     (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG))
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


#endif
void uart3_maix_app_init(void)
{
    uint32 now_ms;

    uart3_maix_hw_init();
    vision_link_init();
    now_ms = heartbeat_get_ms();
    uart3_maix_last_diagnostic_ms = now_ms;
    operational_telemetry_last_ms =
        now_ms - OPERATIONAL_TELEMETRY_PERIOD_MS;
    operational_telemetry_sequence = 0u;
    operational_button_sequence = 0u;
    operational_button_previous = BUTTON_ID_NONE;
    operational_button_last_press = BUTTON_ID_NONE;
#if (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
    chassis_telemetry_last_ms = now_ms;
    chassis_telemetry_sequence = 0u;
    heartbeat_hw_uart_send_string(
        "[mode] uart3 chassis telemetry debug, 0x81/100Hz\r\n");
#elif (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)
    balance_telemetry_last_ms = now_ms;
    balance_telemetry_sequence = 0u;
    heartbeat_hw_uart_send_string(
        "[mode] uart3 balance simple telemetry V6, 0x82/100Hz\r\n");
#else
    heartbeat_hw_uart_send_string(
        "[mode] uart3 operational telemetry, 0x83/100Hz; vision RX\r\n");
#endif
}

void uart3_maix_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    operational_telemetry_process(now_ms);

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
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
        balance_simple_send_diagnostic();
#endif
    }
}
