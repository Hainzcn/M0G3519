#include "imu.h"

#include "heartbeat.h"
#include "imu_hw.h"

typedef enum
{
    IMU_PARSE_SYNC_1 = 0,
    IMU_PARSE_SYNC_2,
    IMU_PARSE_ID,
    IMU_PARSE_LENGTH,
    IMU_PARSE_DATA,
    IMU_PARSE_CHECKSUM,
} imu_parse_state_enum;

#define IMU_ANGLE_DATA_SIZE          (6u)
#define IMU_ACCEL_GYRO_DATA_SIZE     (12u)
#define IMU_PROCESS_MAX_BYTES        (128u)

static imu_angle_t imu_angle_data;
static imu_accel_t imu_accel_data;
static uint8 imu_update_flags;
static uint32 imu_last_angle_ms;
static uint32 imu_last_accel_ms;

static imu_parse_state_enum imu_parse_state;
static uint8 imu_frame_id;
static uint8 imu_frame_length;
static uint8 imu_frame_data[IMU_FRAME_MAX_DATA_SIZE];
static uint8 imu_frame_index;
static uint8 imu_checksum;

static uint32 imu_good_frame_count;
static uint32 imu_bad_frame_count;
static uint32 imu_ignored_frame_count;

static int16 imu_combine_int16(uint8 low, uint8 high)
{
    uint16 raw = (uint16)low | ((uint16)high << 8);
    return (int16)raw;
}

static void imu_parser_reset(void)
{
    imu_parse_state = IMU_PARSE_SYNC_1;
    imu_frame_index = 0u;
    imu_checksum = 0u;
}

static void imu_decode_angle(const uint8 *data)
{
    imu_angle_data.roll =
        (float)imu_combine_int16(data[0], data[1]) * IMU_ANGLE_SCALE_DEG;
    imu_angle_data.pitch =
        (float)imu_combine_int16(data[2], data[3]) * IMU_ANGLE_SCALE_DEG;
    imu_angle_data.yaw =
        (float)imu_combine_int16(data[4], data[5]) * IMU_ANGLE_SCALE_DEG;

    imu_last_angle_ms = heartbeat_get_ms();
    imu_update_flags |= IMU_FLAG_ANGLE;
}

static void imu_decode_accel(const uint8 *data)
{
    imu_accel_data.ax =
        (float)imu_combine_int16(data[0], data[1]) * IMU_ACCEL_SCALE_MPS2;
    imu_accel_data.ay =
        (float)imu_combine_int16(data[2], data[3]) * IMU_ACCEL_SCALE_MPS2;
    imu_accel_data.az =
        (float)imu_combine_int16(data[4], data[5]) * IMU_ACCEL_SCALE_MPS2;

    imu_last_accel_ms = heartbeat_get_ms();
    imu_update_flags |= IMU_FLAG_ACCEL;
}

static void imu_dispatch_frame(void)
{
    switch (imu_frame_id)
    {
        case IMU_FRAME_TYPE_ANGLE:
            if (IMU_ANGLE_DATA_SIZE == imu_frame_length)
            {
                imu_decode_angle(imu_frame_data);
                imu_good_frame_count++;
            }
            else
            {
                imu_bad_frame_count++;
            }
            break;

        case IMU_FRAME_TYPE_ACCEL_GYRO:
            if (IMU_ACCEL_GYRO_DATA_SIZE == imu_frame_length)
            {
                /* The final six gyro bytes are intentionally not converted. */
                imu_decode_accel(imu_frame_data);
                imu_good_frame_count++;
            }
            else
            {
                imu_bad_frame_count++;
            }
            break;

        default:
            imu_ignored_frame_count++;
            break;
    }
}

static void imu_feed_byte(uint8 byte)
{
    switch (imu_parse_state)
    {
        case IMU_PARSE_SYNC_1:
            if (IMU_FRAME_HEADER == byte)
            {
                imu_checksum = byte;
                imu_parse_state = IMU_PARSE_SYNC_2;
            }
            break;

        case IMU_PARSE_SYNC_2:
            if (IMU_FRAME_HEADER == byte)
            {
                imu_checksum = (uint8)(imu_checksum + byte);
                imu_parse_state = IMU_PARSE_ID;
            }
            else
            {
                imu_parser_reset();
            }
            break;

        case IMU_PARSE_ID:
            imu_frame_id = byte;
            imu_checksum = (uint8)(imu_checksum + byte);
            imu_parse_state = IMU_PARSE_LENGTH;
            break;

        case IMU_PARSE_LENGTH:
            if (byte > IMU_FRAME_MAX_DATA_SIZE)
            {
                imu_bad_frame_count++;
                imu_parser_reset();
                break;
            }

            imu_frame_length = byte;
            imu_frame_index = 0u;
            imu_checksum = (uint8)(imu_checksum + byte);
            imu_parse_state =
                (0u == byte) ? IMU_PARSE_CHECKSUM : IMU_PARSE_DATA;
            break;

        case IMU_PARSE_DATA:
            imu_frame_data[imu_frame_index] = byte;
            imu_frame_index++;
            imu_checksum = (uint8)(imu_checksum + byte);

            if (imu_frame_index >= imu_frame_length)
            {
                imu_parse_state = IMU_PARSE_CHECKSUM;
            }
            break;

        case IMU_PARSE_CHECKSUM:
            if (imu_checksum == byte)
            {
                imu_dispatch_frame();
            }
            else
            {
                imu_bad_frame_count++;
            }
            imu_parser_reset();
            break;

        default:
            imu_parser_reset();
            break;
    }
}

static void imu_check_stale(void)
{
    uint32 now_ms = heartbeat_get_ms();

    if ((0u != (imu_update_flags & IMU_FLAG_ANGLE)) &&
        ((now_ms - imu_last_angle_ms) > IMU_STALE_TIMEOUT_MS))
    {
        imu_update_flags &= (uint8)(~IMU_FLAG_ANGLE);
    }

    if ((0u != (imu_update_flags & IMU_FLAG_ACCEL)) &&
        ((now_ms - imu_last_accel_ms) > IMU_STALE_TIMEOUT_MS))
    {
        imu_update_flags &= (uint8)(~IMU_FLAG_ACCEL);
    }
}

void imu_init(void)
{
    imu_hw_init();

    imu_angle_data.roll = 0.0f;
    imu_angle_data.pitch = 0.0f;
    imu_angle_data.yaw = 0.0f;
    imu_accel_data.ax = 0.0f;
    imu_accel_data.ay = 0.0f;
    imu_accel_data.az = 0.0f;
    imu_update_flags = 0u;
    imu_last_angle_ms = 0u;
    imu_last_accel_ms = 0u;
    imu_good_frame_count = 0u;
    imu_bad_frame_count = 0u;
    imu_ignored_frame_count = 0u;
    imu_parser_reset();
}

void imu_process(void)
{
    uint8 byte;
    uint16 bytes = 0u;

    if (0u != imu_hw_take_rx_error())
    {
        imu_bad_frame_count++;
        imu_parser_reset();
    }

    while (bytes < IMU_PROCESS_MAX_BYTES)
    {
        if (0u == imu_hw_read_byte(&byte))
        {
            break;
        }
        imu_feed_byte(byte);
        bytes++;
    }

    imu_check_stale();
}

const imu_angle_t *imu_get_angle(void)
{
    return &imu_angle_data;
}

const imu_accel_t *imu_get_accel(void)
{
    return &imu_accel_data;
}

uint8 imu_get_update_flags(void)
{
    imu_check_stale();
    return imu_update_flags;
}

void imu_get_snapshot(imu_snapshot_t *snapshot)
{
    if (NULL == snapshot)
    {
        return;
    }

    imu_check_stale();
    snapshot->angle = imu_angle_data;
    snapshot->accel = imu_accel_data;
    snapshot->flags = imu_update_flags;
    snapshot->angle_time_ms = imu_last_angle_ms;
    snapshot->accel_time_ms = imu_last_accel_ms;
}

uint8 imu_is_type_ready(uint8 flag)
{
    imu_check_stale();
    return ((imu_update_flags & flag) == flag) ? 1u : 0u;
}

uint8 imu_is_online(void)
{
    return imu_is_type_ready((uint8)(IMU_FLAG_ANGLE | IMU_FLAG_ACCEL));
}

uint32 imu_get_good_frame_count(void)
{
    return imu_good_frame_count;
}

uint32 imu_get_bad_frame_count(void)
{
    return imu_bad_frame_count;
}

uint32 imu_get_ignored_frame_count(void)
{
    return imu_ignored_frame_count;
}
