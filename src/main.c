#include "grayscale_app.h"
#include "emm42_demo_app.h"
#include "heartbeat_app.h"
#include "heartbeat_hw.h"
#include "imu_app.h"
#include "motor.h"
#include "motor_app.h"
#include "oled_app.h"
#include "zf_common_clock.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    emm42_demo_app_init();
    grayscale_app_init();
    motor_app_init();
    motor_app_stop();
    imu_app_init();
    oled_app_init();

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
        imu_app_process();
        grayscale_app_process();
        motor_app_process();
        emm42_demo_app_process();
        heartbeat_app_process();
        oled_app_process();
    }
}
