#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "imu.h"
#include "imu_hw.h"

static uint8 test_stream[128];
static uint16 test_stream_length;
static uint16 test_stream_offset;
static uint32 test_time_ms;

uint32 heartbeat_get_ms(void)
{
    return test_time_ms;
}

void imu_hw_init(void)
{
}

uint8 imu_hw_take_rx_error(void)
{
    return 0u;
}

uint16 imu_hw_read(uint8 *buffer, uint16 max_length)
{
    uint16 count = 0u;

    while ((count < max_length) && (test_stream_offset < test_stream_length))
    {
        buffer[count++] = test_stream[test_stream_offset++];
    }
    return count;
}

static void put_le16(uint8 *buffer, int16 value)
{
    uint16 raw = (uint16)value;
    buffer[0] = (uint8)(raw & 0xFFu);
    buffer[1] = (uint8)(raw >> 8);
}

static uint16 append_frame(uint16 offset, uint8 id, const uint8 *data, uint8 length)
{
    uint8 checksum;
    uint8 i;

    test_stream[offset++] = 0x55u;
    test_stream[offset++] = 0x55u;
    test_stream[offset++] = id;
    test_stream[offset++] = length;
    checksum = (uint8)(0x55u + 0x55u + id + length);

    for (i = 0u; i < length; i++)
    {
        test_stream[offset++] = data[i];
        checksum = (uint8)(checksum + data[i]);
    }
    test_stream[offset++] = checksum;
    return offset;
}

int main(void)
{
    uint8 angle[6] = {0};
    uint8 accel_gyro[12] = {0};
    const imu_angle_t *angle_result;
    const imu_accel_t *accel_result;
    uint16 offset = 0u;

    put_le16(&angle[0], 16384);
    put_le16(&angle[2], -8192);
    put_le16(&angle[4], 0);
    put_le16(&accel_gyro[0], 8192);
    put_le16(&accel_gyro[2], -8192);
    put_le16(&accel_gyro[4], 0);

    offset = append_frame(offset, IMU_FRAME_TYPE_ANGLE, angle, sizeof(angle));
    offset = append_frame(
        offset, IMU_FRAME_TYPE_ACCEL_GYRO, accel_gyro, sizeof(accel_gyro));
    test_stream_length = offset;
    test_stream_offset = 0u;

    imu_init();
    imu_process();

    assert(imu_is_online());
    assert(2u == imu_get_good_frame_count());
    assert(0u == imu_get_bad_frame_count());
    angle_result = imu_get_angle();
    accel_result = imu_get_accel();
    assert(fabsf(angle_result->roll - 90.0f) < 0.001f);
    assert(fabsf(angle_result->pitch + 45.0f) < 0.001f);
    assert(fabsf(angle_result->yaw) < 0.001f);
    assert(fabsf(accel_result->ax - IMU_GRAVITY_MPS2) < 0.001f);
    assert(fabsf(accel_result->ay + IMU_GRAVITY_MPS2) < 0.001f);
    assert(fabsf(accel_result->az) < 0.001f);

    offset = append_frame(0u, IMU_FRAME_TYPE_ANGLE, angle, sizeof(angle));
    test_stream[offset - 1u]++;
    test_stream_length = offset;
    test_stream_offset = 0u;
    imu_process();
    assert(1u == imu_get_bad_frame_count());

    offset = append_frame(0u, 0x02u, NULL, 0u);
    test_stream_length = offset;
    test_stream_offset = 0u;
    imu_process();
    assert(1u == imu_get_ignored_frame_count());

    test_stream_length = 0u;
    test_stream_offset = 0u;
    test_time_ms = IMU_STALE_TIMEOUT_MS + 1u;
    imu_process();
    assert(0u == imu_get_update_flags());

    puts("imu parser tests passed");
    return 0;
}
