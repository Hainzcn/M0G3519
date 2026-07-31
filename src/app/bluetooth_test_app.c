#include "bluetooth_test_app.h"

#include <stdio.h>

#include "bluetooth_hw.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "line_control.h"
#include "motor_app.h"
#include "wheel_speed_control.h"
#include "vision_link.h"

#define BLUETOOTH_TEST_ALIVE_PERIOD_MS      (1000u)
/* Match the 100 Hz wheel-speed loop. Reduce to 50/20 Hz if tx_drop_bytes rises. */
#define CHASSIS_TELEMETRY_HZ                (100u)
#define CHASSIS_TELEMETRY_PERIOD_MS         (1000u / CHASSIS_TELEMETRY_HZ)
#define CHASSIS_TELEMETRY_FRAME_SIZE        (56u)
#define CHASSIS_TELEMETRY_VERSION           (0x02u)
#define CHASSIS_TELEMETRY_TYPE              (0x81u)

#define CHASSIS_FLAG_CONTROL_ACTIVE         (0x01u)
#define CHASSIS_FLAG_LINE_VALID             (0x02u)
#define CHASSIS_FLAG_LINE_LOST              (0x04u)
#define CHASSIS_FLAG_MARKER                  (0x08u)
#define CHASSIS_FLAG_LEFT_SATURATED          (0x10u)
#define CHASSIS_FLAG_RIGHT_SATURATED         (0x20u)
#define CHASSIS_FLAG_KINEMATICS_VALID        (0x40u)
#define CHASSIS_FLAG_UART_RX_OVERFLOW        (0x80u)

static uint32 bluetooth_test_last_alive_ms;
static uint32 chassis_telemetry_last_ms;
static uint16 chassis_telemetry_sequence;

static void vision_link_send_diagnostic(void)
{
    char message[192];
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

static void chassis_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFu);
    buffer[1] = (uint8)(value >> 8u);
}

static void chassis_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFu);
    buffer[1] = (uint8)((value >> 8u) & 0xFFu);
    buffer[2] = (uint8)((value >> 16u) & 0xFFu);
    buffer[3] = (uint8)((value >> 24u) & 0xFFu);
}

static int16 chassis_float_to_i16(float value, float scale)
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

static void chassis_write_scaled_i16(uint8 *buffer, float value, float scale)
{
    chassis_write_u16_le(buffer,
        (uint16)chassis_float_to_i16(value, scale));
}

static uint16 chassis_crc16_ccitt_false(const uint8 *data, uint16 length)
{
    uint16 crc = 0xFFFFu;
    uint16 index;
    uint8 bit;

    for (index = 0u; index < length; index++)
    {
        crc ^= (uint16)data[index] << 8u;
        for (bit = 0u; bit < 8u; bit++)
        {
            if (0u != (crc & 0x8000u))
            {
                crc = (uint16)((crc << 1u) ^ 0x1021u);
            }
            else
            {
                crc <<= 1u;
            }
        }
    }
    return crc;
}

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
    if (0u != bluetooth_hw_get_rx_overflow_count())
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
    uint32 rx_overflow = bluetooth_hw_get_rx_overflow_count();
    uint16 crc;

    imu_get_snapshot(&imu);

    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = CHASSIS_TELEMETRY_VERSION;
    frame[3] = CHASSIS_TELEMETRY_TYPE;
    frame[4] = CHASSIS_TELEMETRY_FRAME_SIZE;
    frame[5] = (uint8)mode;
    frame[6] = chassis_telemetry_flags(mode, line, wheel);
    frame[7] = 0u;
    chassis_write_u16_le(&frame[8], chassis_telemetry_sequence);
    chassis_write_u32_le(&frame[10], now_ms);
    chassis_write_scaled_i16(&frame[14], wheel->planned_speed_mps, 1000.0f);
    chassis_write_scaled_i16(&frame[16], wheel->planned_accel_mps2, 1000.0f);
    chassis_write_scaled_i16(&frame[18], wheel->left_target_rpm, 10.0f);
    chassis_write_scaled_i16(&frame[20], wheel->right_target_rpm, 10.0f);
    chassis_write_scaled_i16(&frame[22], wheel->left_measured_rpm, 10.0f);
    chassis_write_scaled_i16(&frame[24], wheel->right_measured_rpm, 10.0f);
    chassis_write_scaled_i16(&frame[26], wheel->left_feedforward_pwm, 1.0f);
    chassis_write_scaled_i16(&frame[28], wheel->right_feedforward_pwm, 1.0f);
    chassis_write_scaled_i16(&frame[30], wheel->left_feedback_pwm, 1.0f);
    chassis_write_scaled_i16(&frame[32], wheel->right_feedback_pwm, 1.0f);
    chassis_write_scaled_i16(&frame[34], (float)wheel->left_duty, 1.0f);
    chassis_write_scaled_i16(&frame[36], (float)wheel->right_duty, 1.0f);
    chassis_write_scaled_i16(&frame[38], wheel->measured_speed_mps, 1000.0f);
    chassis_write_scaled_i16(&frame[40], wheel->measured_accel_mps2, 1000.0f);
    chassis_write_scaled_i16(&frame[42], line->error, 1.0f);
    chassis_write_u32_le(&frame[44], bluetooth_hw_get_tx_drop_count());
    chassis_write_u16_le(&frame[48],
        (uint16)((rx_overflow > 65535u) ? 65535u : rx_overflow));
    if (0u != (imu.flags & IMU_FLAG_ACCEL))
    {
        chassis_write_scaled_i16(&frame[50], imu.accel.ax, 1000.0f);
    }
    else
    {
        chassis_write_u16_le(&frame[50], 0x8000u);
    }
    frame[52] = imu.flags;
    frame[53] = 0u;
    crc = chassis_crc16_ccitt_false(frame, 54u);
    chassis_write_u16_le(&frame[54], crc);

    (void)bluetooth_hw_write_atomic(frame, CHASSIS_TELEMETRY_FRAME_SIZE);
    chassis_telemetry_sequence++;
}

void bluetooth_test_app_init(void)
{
    bluetooth_hw_init();
    vision_link_init();
    bluetooth_test_last_alive_ms = heartbeat_get_ms();
    chassis_telemetry_last_ms = bluetooth_test_last_alive_ms;
    chassis_telemetry_sequence = 0u;

    bluetooth_hw_send_string("[link] ready,115200,telemetry=100Hz\r\n");
    heartbeat_hw_uart_send_string(
        "[mode] uart3 vision+telemetry, chassis disabled\r\n");
}

void bluetooth_test_app_process(void)
{
    uint32 now_ms;

    bluetooth_hw_tx_pump();

    now_ms = heartbeat_get_ms();
    if ((now_ms - chassis_telemetry_last_ms) >=
        CHASSIS_TELEMETRY_PERIOD_MS)
    {
        chassis_telemetry_last_ms = now_ms;
        chassis_telemetry_send(now_ms);
    }

    if ((now_ms - bluetooth_test_last_alive_ms) >=
        BLUETOOTH_TEST_ALIVE_PERIOD_MS)
    {
        bluetooth_test_last_alive_ms = now_ms;
        bluetooth_hw_send_string("[link] alive\r\n");
        vision_link_send_diagnostic();
    }

    bluetooth_hw_tx_pump();
}
