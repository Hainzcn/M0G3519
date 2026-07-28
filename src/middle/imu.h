#ifndef IMU_H_
#define IMU_H_

#include "zf_common_typedef.h"

/*
 * Single-axis IMU UART middle layer.
 *
 * Read frame (5 bytes): 0x5A | TYPE | DATAL | DATAH | SUM
 * Write frame (hardware layer): 55 AA ADDR DATAL DATAH
 *
 * See docs/数据手册(串口通信).pdf
 */

#define IMU_FRAME_HEADER           (0x5A)
#define IMU_FRAME_SIZE             (5)

#define IMU_TYPE_GYRO              (0xAA)
#define IMU_TYPE_ANGLE             (0xBB)

#define IMU_FLAG_GYRO              (0x01u)
#define IMU_FLAG_ANGLE             (0x02u)

#define IMU_GYRO_SCALE_DPS         (2000.0f)
#define IMU_GYRO_RANGE_DPS         (400.0f)
#define IMU_ANGLE_RANGE_DEG        (180.0f)
#define IMU_STALE_TIMEOUT_MS       (200u)

typedef struct
{
    float wz;
} imu_gyro_t;

typedef struct
{
    float yaw;
} imu_angle_t;

typedef struct
{
    imu_gyro_t  gyro;
    imu_angle_t angle;
    uint8       flags;
} imu_snapshot_t;

void imu_init(void);
void imu_process(void);

const imu_gyro_t  *imu_get_gyro(void);
const imu_angle_t *imu_get_angle(void);
uint8 imu_get_update_flags(void);
void imu_get_snapshot(imu_snapshot_t *snapshot);
uint8 imu_is_type_ready(uint8 flag);
uint8 imu_is_online(void);

#endif
