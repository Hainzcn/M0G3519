#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "button.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "uart3_maix_app.h"
#include "uart3_maix_hw.h"
#include "vision_link.h"
#include "wheel_speed_control.h"

#define TEST_FRAME_SIZE (24u)

static uint32 mock_now_ms;
static button_id_t mock_button;
static imu_snapshot_t mock_imu;
static wheel_speed_control_status_t mock_wheel;
static uint8 captured_frame[TEST_FRAME_SIZE];
static uint16 captured_length;
static uint32 captured_count;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

button_id_t button_get_active(void)
{
    return mock_button;
}

void imu_get_snapshot(imu_snapshot_t *snapshot)
{
    *snapshot = mock_imu;
}

const wheel_speed_control_status_t *wheel_speed_control_get_status(void)
{
    return &mock_wheel;
}

void uart3_maix_hw_init(void)
{
}

uint16 uart3_maix_hw_write_atomic(const uint8 *data, uint16 length)
{
    assert(length == TEST_FRAME_SIZE);
    memcpy(captured_frame, data, length);
    captured_length = length;
    captured_count++;
    return length;
}

void uart3_maix_hw_tx_pump(void)
{
}

void vision_link_init(void)
{
}

void vision_link_get_status(vision_link_status_t *status)
{
    memset(status, 0, sizeof(*status));
}

static uint16 read_u16(const uint8 *data)
{
    return (uint16)data[0] | ((uint16)data[1] << 8u);
}

static uint32 read_u32(const uint8 *data)
{
    return (uint32)data[0] |
        ((uint32)data[1] << 8u) |
        ((uint32)data[2] << 16u) |
        ((uint32)data[3] << 24u);
}

static uint16 crc16_ccitt_false(const uint8 *data, uint16 length)
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

static void assert_frame_crc(void)
{
    assert(captured_length == TEST_FRAME_SIZE);
    assert(read_u16(&captured_frame[22]) ==
           crc16_ccitt_false(captured_frame, 22u));
}

static void test_boot_frame_is_sent_immediately(void)
{
    memset(&mock_imu, 0, sizeof(mock_imu));
    memset(&mock_wheel, 0, sizeof(mock_wheel));
    mock_now_ms = 0u;
    mock_button = BUTTON_ID_NONE;
    captured_count = 0u;

    uart3_maix_app_init();
    uart3_maix_app_process();

    assert(captured_count == 1u);
    assert(captured_frame[0] == 0xA5u);
    assert(captured_frame[1] == 0x5Au);
    assert(captured_frame[2] == 0x01u);
    assert(captured_frame[3] == 0x83u);
    assert(captured_frame[4] == TEST_FRAME_SIZE);
    assert(captured_frame[5] == 0u);
    assert(captured_frame[6] == BUTTON_ID_NONE);
    assert(captured_frame[7] == BUTTON_ID_NONE);
    assert(read_u16(&captured_frame[8]) == 0u);
    assert(read_u16(&captured_frame[10]) == 0u);
    assert(read_u32(&captured_frame[12]) == 0u);
    assert(read_u16(&captured_frame[16]) == 0x8000u);
    assert_frame_crc();
}

static void test_button_imu_and_encoder_payload(void)
{
    mock_now_ms = 10u;
    mock_button = BUTTON_ID_SW3;
    mock_imu.flags = IMU_FLAG_ACCEL;
    mock_imu.accel.ax = 1.25f;
    mock_imu.accel_time_ms = mock_now_ms;
    mock_wheel.left_measured_rpm = -12.3f;
    mock_wheel.right_measured_rpm = 45.6f;

    uart3_maix_app_process();

    assert(captured_count == 2u);
    assert(captured_frame[5] == 0x01u);
    assert(captured_frame[6] == BUTTON_ID_SW3);
    assert(captured_frame[7] == BUTTON_ID_SW3);
    assert(read_u16(&captured_frame[8]) == 1u);
    assert(read_u16(&captured_frame[10]) == 1u);
    assert(read_u32(&captured_frame[12]) == mock_now_ms);
    assert((int16)read_u16(&captured_frame[16]) == 1250);
    assert((int16)read_u16(&captured_frame[18]) == -123);
    assert((int16)read_u16(&captured_frame[20]) == 456);
    assert_frame_crc();

    mock_now_ms = 20u;
    mock_button = BUTTON_ID_NONE;
    mock_imu.accel_time_ms = mock_now_ms;
    uart3_maix_app_process();
    assert(captured_frame[6] == BUTTON_ID_SW3);
    assert(captured_frame[7] == BUTTON_ID_NONE);
    assert(read_u16(&captured_frame[8]) == 1u);
    assert(read_u16(&captured_frame[10]) == 2u);
    assert_frame_crc();
}

static void test_stale_imu_is_marked_invalid(void)
{
    mock_now_ms = 130u;
    mock_imu.accel_time_ms = 20u;
    uart3_maix_app_process();

    assert(captured_count == 4u);
    assert((captured_frame[5] & 0x01u) == 0u);
    assert(read_u16(&captured_frame[16]) == 0x8000u);
    assert_frame_crc();
}

int main(void)
{
    test_boot_frame_is_sent_immediately();
    test_button_imu_and_encoder_payload();
    test_stale_imu_is_marked_invalid();
    puts("UART3 operational telemetry tests passed");
    return 0;
}
