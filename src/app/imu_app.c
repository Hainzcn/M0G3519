#include "imu_app.h"

#include <stdio.h>

#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "imu_hw.h"

#define IMU_APP_BOOT_DELAY_MS          (500)
#define IMU_APP_CMD_DELAY_MS           (100)
#define IMU_APP_DEBUG_PERIOD_MS        (1000)
#define IMU_APP_WAIT_DEBUG_PERIOD_MS   (2000)

#define IMU_REG_KEY                    (0x13)
#define IMU_REG_SAVE                   (0x00)
#define IMU_REG_YAW_ZERO               (0x15)

#ifndef IMU_APP_YAW_ZERO_ON_BOOT
#define IMU_APP_YAW_ZERO_ON_BOOT       (0)
#endif

typedef enum
{
    IMU_APP_STATE_WAIT_BOOT = 0,
    IMU_APP_STATE_YAW_UNLOCK,
    IMU_APP_STATE_YAW_CMD,
    IMU_APP_STATE_YAW_SAVE,
    IMU_APP_STATE_RUNNING,
} imu_app_state_enum;

static imu_app_state_enum imu_app_state;
static uint32 imu_app_state_start_ms;
static uint32 imu_app_last_print_ms;
static uint32 imu_app_last_wait_print_ms;
static uint32 imu_app_print_count;

void imu_app_init(void)
{
    imu_init();

    imu_app_state              = IMU_APP_STATE_WAIT_BOOT;
    imu_app_state_start_ms     = heartbeat_get_ms();
    imu_app_last_print_ms      = 0;
    imu_app_last_wait_print_ms = 0;
    imu_app_print_count        = 0;
}

static uint8 imu_app_has_required_data(void)
{
    return imu_is_online();
}

static void imu_app_print_wait_if_needed(uint32 now_ms)
{
    char message[48];
    uint8 flags;

    if ((now_ms - imu_app_last_wait_print_ms) < IMU_APP_WAIT_DEBUG_PERIOD_MS)
    {
        return;
    }

    imu_app_last_wait_print_ms = now_ms;
    flags = imu_get_update_flags();
    snprintf(message, sizeof(message),
             "[imu] 0,wait,flags=0x%02X\r\n", (unsigned int)flags);
    heartbeat_hw_uart_send_string(message);
}

static void imu_app_run_setup_state_machine(void)
{
    uint32 now_ms = heartbeat_get_ms();

    switch (imu_app_state)
    {
        case IMU_APP_STATE_WAIT_BOOT:
            if ((now_ms - imu_app_state_start_ms) < IMU_APP_BOOT_DELAY_MS)
            {
                break;
            }

            imu_hw_rx_enable();

#if IMU_APP_YAW_ZERO_ON_BOOT
            imu_app_state          = IMU_APP_STATE_YAW_UNLOCK;
            imu_app_state_start_ms = now_ms;
#else
            imu_app_state = IMU_APP_STATE_RUNNING;
#endif
            break;

        case IMU_APP_STATE_YAW_UNLOCK:
            imu_hw_write_reg(IMU_REG_KEY, (int16)0x8E5F);
            imu_app_state          = IMU_APP_STATE_YAW_CMD;
            imu_app_state_start_ms = now_ms;
            break;

        case IMU_APP_STATE_YAW_CMD:
            if ((now_ms - imu_app_state_start_ms) < IMU_APP_CMD_DELAY_MS)
            {
                break;
            }

            imu_hw_write_reg(IMU_REG_YAW_ZERO, 0);
            imu_app_state          = IMU_APP_STATE_YAW_SAVE;
            imu_app_state_start_ms = now_ms;
            break;

        case IMU_APP_STATE_YAW_SAVE:
            if ((now_ms - imu_app_state_start_ms) < IMU_APP_CMD_DELAY_MS)
            {
                break;
            }

            imu_hw_write_reg(IMU_REG_SAVE, 0);
            imu_app_state = IMU_APP_STATE_RUNNING;
            break;

        case IMU_APP_STATE_RUNNING:
        default:
            break;
    }
}

void imu_app_process(void)
{
    char message[64];
    const imu_angle_t *angle;
    const imu_gyro_t  *gyro;
    uint32 now_ms;

    imu_app_run_setup_state_machine();

    if (IMU_APP_STATE_RUNNING != imu_app_state)
    {
        return;
    }

    now_ms = heartbeat_get_ms();
    imu_process();

    if (!imu_app_has_required_data())
    {
        imu_app_print_wait_if_needed(now_ms);
        return;
    }

    if ((now_ms - imu_app_last_print_ms) < IMU_APP_DEBUG_PERIOD_MS)
    {
        return;
    }

    imu_app_last_print_ms = now_ms;
    imu_app_print_count ++;

    angle = imu_get_angle();
    gyro  = imu_get_gyro();

    snprintf(message, sizeof(message),
             "[imu] %u,%.2f,%.2f\r\n",
             (unsigned int)imu_app_print_count,
             (double)angle->yaw, (double)gyro->wz);
    heartbeat_hw_uart_send_string(message);
}
