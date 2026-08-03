#ifndef IMU_H_
#define IMU_H_

#include "zf_common_typedef.h"

/*
 * ATK-MS901M stream parser.
 *
 * Frame: 55 55 ID LEN DATA[LEN] SUM
 * SUM is the low byte of the sum from the first header through the last data
 * byte. Only attitude (ID 0x01) and acceleration from gyro/accel (ID 0x03)
 * are converted. Other valid frames are skipped without losing sync.
 */

#define IMU_FRAME_HEADER            (0x55u)
#define IMU_FRAME_TYPE_ANGLE        (0x01u)
#define IMU_FRAME_TYPE_ACCEL_GYRO   (0x03u)
#define IMU_FRAME_MAX_DATA_SIZE     (32u)

#define IMU_FLAG_ANGLE              (0x01u)
#define IMU_FLAG_ACCEL              (0x02u)

/* Must match the module configuration. ATK-MS901M defaults to +/-4 g. */
#define IMU_ACCEL_FSR_G             (4.0f)
#define IMU_GRAVITY_MPS2            (9.80665f)
#define IMU_ANGLE_SCALE_DEG         (180.0f / 32768.0f)
#define IMU_ACCEL_SCALE_MPS2        \
    (IMU_ACCEL_FSR_G * IMU_GRAVITY_MPS2 / 32768.0f)

/* Module active report rate; must match ATK-MS901M configuration. */
#define IMU_REPORT_RATE_HZ          (100u)

/* Allow ~10 sample periods before clearing a cached frame type. */
#define IMU_STALE_TIMEOUT_MS        ((1000u + IMU_REPORT_RATE_HZ - 1u) / \
                                     IMU_REPORT_RATE_HZ * 10u)

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} imu_angle_t;

typedef struct
{
    float ax;
    float ay;
    float az;
} imu_accel_t;

typedef struct
{
    imu_angle_t angle;
    imu_accel_t accel;
    uint8       flags;
    uint32      angle_time_ms;
    uint32      accel_time_ms;
} imu_snapshot_t;

void imu_init(void);
void imu_process(void);
uint8 imu_configure_active_stream(void);

const imu_angle_t *imu_get_angle(void);
const imu_accel_t *imu_get_accel(void);
uint8 imu_get_update_flags(void);
void imu_get_snapshot(imu_snapshot_t *snapshot);
uint8 imu_is_type_ready(uint8 flag);
uint8 imu_is_online(void);

uint32 imu_get_good_frame_count(void);
uint32 imu_get_bad_frame_count(void);
uint32 imu_get_ignored_frame_count(void);

#endif
