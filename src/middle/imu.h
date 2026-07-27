#ifndef IMU_H_
#define IMU_H_

#include "zf_common_typedef.h"

/*
 * ???? IMU ????��???��??
 *
 * ?????0x5A | TYPE | 8 data bytes | SUM
 * ��???hardware ????55 AA ADDR DATAL DATAH
 */

#define IMU_ENABLE_GYRO            (0)
#define IMU_ENABLE_ANGLE           (1)
#define IMU_ENABLE_ACCEL           (1)
#define IMU_ENABLE_QUAT            (0)

#define IMU_FRAME_HEADER           (0x5A)
#define IMU_FRAME_SIZE             (11)

#define IMU_TYPE_GYRO              (0xAA)
#define IMU_TYPE_ANGLE             (0xBB)
#define IMU_TYPE_ACCEL             (0xCC)
#define IMU_TYPE_QUAT              (0xDD)

#define IMU_FLAG_GYRO              (0x01u)
#define IMU_FLAG_ANGLE             (0x02u)
#define IMU_FLAG_ACCEL             (0x04u)
#define IMU_FLAG_QUAT              (0x08u)

typedef struct
{
    float wx;
    float wy;
    float wz;
} imu_gyro_t;

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
    float q0;
    float q1;
    float q2;
    float q3;
} imu_quat_t;

typedef struct
{
    imu_gyro_t  gyro;
    imu_angle_t angle;
    imu_accel_t accel;
    imu_quat_t  quat;
    uint8       flags;
} imu_snapshot_t;

void imu_init(void);
void imu_process(void);

const imu_gyro_t  *imu_get_gyro(void);
const imu_angle_t *imu_get_angle(void);
const imu_accel_t *imu_get_accel(void);
const imu_quat_t  *imu_get_quat(void);
uint8 imu_get_update_flags(void);
void imu_get_snapshot(imu_snapshot_t *snapshot);
uint8 imu_is_type_ready(uint8 flag);

#endif
