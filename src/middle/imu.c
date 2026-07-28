#include "imu.h"

#include "imu_hw.h"

typedef enum
{
    IMU_PARSE_SYNC = 0,
    IMU_PARSE_TYPE,
    IMU_PARSE_DATA,
    IMU_PARSE_SUM,
} imu_parse_state_enum;

#define IMU_FRAME_DATA_SIZE        (2)
#define IMU_PROCESS_MAX_BYTES      (0)   /* 0 = drain entire RX ring each call */

static imu_gyro_t imu_gyro_data;
static imu_angle_t imu_angle_data;
static uint8 imu_update_flags;

static imu_parse_state_enum imu_parse_state;
static uint8 imu_frame_type;
static uint8 imu_frame_data[IMU_FRAME_DATA_SIZE];
static uint8 imu_frame_index;
static uint8 imu_checksum;

static int16 imu_combine_int16(uint8 low, uint8 high)
{
    return (int16)((int16)((int16)high << 8) | low);
}

static uint8 imu_calc_checksum(uint8 type, const uint8 *data)
{
    uint8 sum = (uint8)(IMU_FRAME_HEADER + type + data[0] + data[1]);

    return sum;
}

static uint8 imu_is_valid_type(uint8 type)
{
    return (uint8)((IMU_TYPE_GYRO == type) || (IMU_TYPE_ANGLE == type));
}

static uint8 imu_gyro_in_range(float wz)
{
    const float limit = IMU_GYRO_RANGE_DPS;

    return ((wz < -limit) || (limit < wz)) ? 0u : 1u;
}

static uint8 imu_angle_in_range(float yaw)
{
    const float limit = IMU_ANGLE_RANGE_DEG;

    return ((yaw < -limit) || (limit < yaw)) ? 0u : 1u;
}

static void imu_decode_gyro(const uint8 *data)
{
    float wz;
    int16 raw_wz = imu_combine_int16(data[0], data[1]);

    wz = (float)raw_wz / 32768.0f * IMU_GYRO_SCALE_DPS;

    if (!imu_gyro_in_range(wz))
    {
        return;
    }

    imu_gyro_data.wz = wz;
    imu_update_flags |= IMU_FLAG_GYRO;
}

static void imu_decode_angle(const uint8 *data)
{
    float yaw;
    int16 raw_yaw = imu_combine_int16(data[0], data[1]);

    yaw = (float)raw_yaw / 32768.0f * IMU_ANGLE_RANGE_DEG;

    if (!imu_angle_in_range(yaw))
    {
        return;
    }

    imu_angle_data.yaw = yaw;
    imu_update_flags |= IMU_FLAG_ANGLE;
}

static void imu_handle_frame(uint8 type, const uint8 *data)
{
    switch (type)
    {
        case IMU_TYPE_GYRO:
            imu_decode_gyro(data);
            break;

        case IMU_TYPE_ANGLE:
            imu_decode_angle(data);
            break;

        default:
            break;
    }
}

static void imu_feed_byte(uint8 byte)
{
    switch (imu_parse_state)
    {
        case IMU_PARSE_SYNC:
            if (IMU_FRAME_HEADER == byte)
            {
                imu_parse_state = IMU_PARSE_TYPE;
            }
            break;

        case IMU_PARSE_TYPE:
            if (imu_is_valid_type(byte))
            {
                imu_frame_type  = byte;
                imu_frame_index = 0;
                imu_parse_state = IMU_PARSE_DATA;
            }
            else if (IMU_FRAME_HEADER == byte)
            {
                imu_parse_state = IMU_PARSE_TYPE;
            }
            else
            {
                imu_parse_state = IMU_PARSE_SYNC;
            }
            break;

        case IMU_PARSE_DATA:
            imu_frame_data[imu_frame_index] = byte;
            imu_frame_index++;

            if (IMU_FRAME_DATA_SIZE <= imu_frame_index)
            {
                imu_checksum = imu_calc_checksum(imu_frame_type, imu_frame_data);
                imu_parse_state = IMU_PARSE_SUM;
            }
            break;

        case IMU_PARSE_SUM:
            if (imu_checksum == byte)
            {
                imu_handle_frame(imu_frame_type, imu_frame_data);
            }
            imu_parse_state = IMU_PARSE_SYNC;
            break;

        default:
            imu_parse_state = IMU_PARSE_SYNC;
            break;
    }
}

void imu_init(void)
{
    imu_hw_init();

    imu_gyro_data.wz   = 0.0f;
    imu_angle_data.yaw = 0.0f;

    imu_update_flags = 0;
    imu_parse_state  = IMU_PARSE_SYNC;
    imu_frame_index  = 0;
}

void imu_process(void)
{
    uint8 byte;
    uint8 count = 0;

    while (imu_hw_read_byte(&byte))
    {
        imu_feed_byte(byte);

        if ((0u != IMU_PROCESS_MAX_BYTES) && (IMU_PROCESS_MAX_BYTES <= ++count))
        {
            break;
        }
    }
}

const imu_gyro_t *imu_get_gyro(void)
{
    return &imu_gyro_data;
}

const imu_angle_t *imu_get_angle(void)
{
    return &imu_angle_data;
}

uint8 imu_get_update_flags(void)
{
    return imu_update_flags;
}

void imu_get_snapshot(imu_snapshot_t *snapshot)
{
    if (NULL == snapshot)
    {
        return;
    }

    snapshot->gyro  = imu_gyro_data;
    snapshot->angle = imu_angle_data;
    snapshot->flags = imu_update_flags;
}

uint8 imu_is_type_ready(uint8 flag)
{
    return ((imu_update_flags & flag) != 0u) ? 1u : 0u;
}
