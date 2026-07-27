#include "imu.h"

#include "imu_hw.h"

typedef enum
{
    IMU_PARSE_SYNC = 0,
    IMU_PARSE_TYPE,
    IMU_PARSE_DATA,
    IMU_PARSE_SUM,
} imu_parse_state_enum;

#define IMU_TYPE_REG               (0xEE)

#define IMU_PROCESS_MAX_BYTES      (0)   /* 0 = drain entire RX ring each call */

static imu_gyro_t imu_gyro_data;
static imu_angle_t imu_angle_data;
static imu_accel_t imu_accel_data;
static imu_quat_t imu_quat_data;
static uint8 imu_update_flags;

static imu_parse_state_enum imu_parse_state;
static uint8 imu_frame_type;
static uint8 imu_frame_data[8];
static uint8 imu_frame_index;
static uint8 imu_checksum;

static int16 imu_combine_int16(uint8 low, uint8 high)
{
    return (int16)((int16)((int16)high << 8) | low);
}

static uint8 imu_calc_checksum(uint8 type, const uint8 *data)
{
    uint8 sum = IMU_FRAME_HEADER + type;
    uint8 i;

    for (i = 0; i < 8; i++)
    {
        sum = (uint8)(sum + data[i]);
    }

    return sum;
}

static uint8 imu_is_valid_type(uint8 type)
{
    return (uint8)((IMU_TYPE_GYRO == type) || (IMU_TYPE_ANGLE == type) ||
                   (IMU_TYPE_ACCEL == type) || (IMU_TYPE_QUAT == type) ||
                   (IMU_TYPE_REG == type));
}

static uint8 imu_gyro_in_range(float wx, float wy, float wz)
{
    const float limit = 2000.0f;

    if ((wx < -limit) || (limit < wx))
    {
        return 0u;
    }
    if ((wy < -limit) || (limit < wy))
    {
        return 0u;
    }
    if ((wz < -limit) || (limit < wz))
    {
        return 0u;
    }

    return 1u;
}

static uint8 imu_angle_in_range(float roll, float pitch, float yaw)
{
    const float limit = 180.0f;

    if ((roll < -limit) || (limit < roll))
    {
        return 0u;
    }
    if ((pitch < -limit) || (limit < pitch))
    {
        return 0u;
    }
    if ((yaw < -limit) || (limit < yaw))
    {
        return 0u;
    }

    return 1u;
}

static void imu_decode_gyro(const uint8 *data)
{
#if IMU_ENABLE_GYRO
    float wx;
    float wy;
    float wz;
    int16 raw_wx = imu_combine_int16(data[0], data[1]);
    int16 raw_wy = imu_combine_int16(data[2], data[3]);
    int16 raw_wz = imu_combine_int16(data[4], data[5]);

    wx = (float)raw_wx / 32768.0f * 2000.0f;
    wy = (float)raw_wy / 32768.0f * 2000.0f;
    wz = (float)raw_wz / 32768.0f * 2000.0f;

    if (!imu_gyro_in_range(wx, wy, wz))
    {
        return;
    }

    imu_gyro_data.wx = wx;
    imu_gyro_data.wy = wy;
    imu_gyro_data.wz = wz;
    imu_update_flags |= IMU_FLAG_GYRO;
#else
    (void)data;
#endif
}

static void imu_decode_angle(const uint8 *data)
{
#if IMU_ENABLE_ANGLE
    float roll;
    float pitch;
    float yaw;
    int16 raw_roll  = imu_combine_int16(data[0], data[1]);
    int16 raw_pitch = imu_combine_int16(data[2], data[3]);
    int16 raw_yaw   = imu_combine_int16(data[4], data[5]);

    roll  = (float)raw_roll  / 32768.0f * 180.0f;
    pitch = (float)raw_pitch / 32768.0f * 180.0f;
    yaw   = (float)raw_yaw   / 32768.0f * 180.0f;

    if (!imu_angle_in_range(roll, pitch, yaw))
    {
        return;
    }

    imu_angle_data.roll  = roll;
    imu_angle_data.pitch = pitch;
    imu_angle_data.yaw   = yaw;
    imu_update_flags |= IMU_FLAG_ANGLE;
#else
    (void)data;
#endif
}

static void imu_decode_accel(const uint8 *data)
{
#if IMU_ENABLE_ACCEL
    int16 raw_ax = imu_combine_int16(data[0], data[1]);
    int16 raw_ay = imu_combine_int16(data[2], data[3]);
    int16 raw_az = imu_combine_int16(data[4], data[5]);
    const float gravity = 9.8f;

    imu_accel_data.ax = (float)raw_ax / 32768.0f * 16.0f * gravity;
    imu_accel_data.ay = (float)raw_ay / 32768.0f * 16.0f * gravity;
    imu_accel_data.az = (float)raw_az / 32768.0f * 16.0f * gravity;
    imu_update_flags |= IMU_FLAG_ACCEL;
#else
    (void)data;
#endif
}

static void imu_decode_quat(const uint8 *data)
{
#if IMU_ENABLE_QUAT
    int16 raw_q0 = imu_combine_int16(data[0], data[1]);
    int16 raw_q1 = imu_combine_int16(data[2], data[3]);
    int16 raw_q2 = imu_combine_int16(data[4], data[5]);
    int16 raw_q3 = imu_combine_int16(data[6], data[7]);

    imu_quat_data.q0 = (float)raw_q0 / 32768.0f;
    imu_quat_data.q1 = (float)raw_q1 / 32768.0f;
    imu_quat_data.q2 = (float)raw_q2 / 32768.0f;
    imu_quat_data.q3 = (float)raw_q3 / 32768.0f;
    imu_update_flags |= IMU_FLAG_QUAT;
#else
    (void)data;
#endif
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

        case IMU_TYPE_ACCEL:
            imu_decode_accel(data);
            break;

        case IMU_TYPE_QUAT:
            imu_decode_quat(data);
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

            if (8u <= imu_frame_index)
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

    imu_gyro_data.wx  = 0.0f;
    imu_gyro_data.wy  = 0.0f;
    imu_gyro_data.wz  = 0.0f;
    imu_angle_data.roll  = 0.0f;
    imu_angle_data.pitch = 0.0f;
    imu_angle_data.yaw   = 0.0f;
    imu_accel_data.ax = 0.0f;
    imu_accel_data.ay = 0.0f;
    imu_accel_data.az = 0.0f;
    imu_quat_data.q0 = 0.0f;
    imu_quat_data.q1 = 0.0f;
    imu_quat_data.q2 = 0.0f;
    imu_quat_data.q3 = 0.0f;

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

const imu_accel_t *imu_get_accel(void)
{
    return &imu_accel_data;
}

const imu_quat_t *imu_get_quat(void)
{
    return &imu_quat_data;
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
    snapshot->accel = imu_accel_data;
    snapshot->quat  = imu_quat_data;
    snapshot->flags = imu_update_flags;
}

uint8 imu_is_type_ready(uint8 flag)
{
    return ((imu_update_flags & flag) != 0u) ? 1u : 0u;
}
