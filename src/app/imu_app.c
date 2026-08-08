#include "imu_app.h"

#include <stdio.h>

#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "imu.h"
#include "imu_hw.h"

#define IMU_APP_DEBUG_PERIOD_MS        (1000u)
#define IMU_APP_WAIT_DEBUG_PERIOD_MS   (2000u)

static uint32 imu_app_last_print_ms;
static uint32 imu_app_last_wait_print_ms;
static uint32 imu_app_print_count;

void imu_app_init(void)
{
    imu_init();
    imu_hw_rx_enable();
    if (0u == imu_configure_active_stream())
    {
        heartbeat_hw_uart_send_string("[imu] stream config failed\r\n");
    }

    imu_app_last_print_ms = 0u;
    imu_app_last_wait_print_ms = 0u;
    imu_app_print_count = 0u;
}

static void imu_app_print_wait_if_needed(uint32 now_ms)
{
    char message[96];

    if ((now_ms - imu_app_last_wait_print_ms) <
        IMU_APP_WAIT_DEBUG_PERIOD_MS)
    {
        return;
    }

    imu_app_last_wait_print_ms = now_ms;
    snprintf(message, sizeof(message),
             "[imu] wait,f=0x%02X,good=%lu,bad=%lu,ovf=%lu\r\n",
             (unsigned int)imu_get_update_flags(),
             (unsigned long)imu_get_good_frame_count(),
             (unsigned long)imu_get_bad_frame_count(),
             (unsigned long)imu_hw_get_overflow_count());
    heartbeat_hw_uart_send_string(message);
}

void imu_app_process(void)
{
    char message[144];
    const imu_angle_t *angle;
    const imu_accel_t *accel;
    uint32 now_ms = heartbeat_get_ms();

    imu_process();

    if (!imu_is_online())
    {
        imu_app_print_wait_if_needed(now_ms);
        return;
    }

    if ((now_ms - imu_app_last_print_ms) < IMU_APP_DEBUG_PERIOD_MS)
    {
        return;
    }

    imu_app_last_print_ms = now_ms;
    imu_app_print_count++;
    angle = imu_get_angle();
    accel = imu_get_accel();

    snprintf(message, sizeof(message),
             "[imu] %lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,bad=%lu,ovf=%lu\r\n",
             (unsigned long)imu_app_print_count,
             (double)angle->roll,
             (double)angle->pitch,
             (double)angle->yaw,
             (double)accel->ax,
             (double)accel->ay,
             (double)accel->az,
             (unsigned long)imu_get_bad_frame_count(),
             (unsigned long)imu_hw_get_overflow_count());
    heartbeat_hw_uart_send_string(message);
}
