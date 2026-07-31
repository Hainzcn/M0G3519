#include "button_app.h"
#include "control_config.h"
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
#include "emm42_demo_app.h"
#endif
#include "uart3_maix_app.h"
#include "grayscale_app.h"
#include "heartbeat_app.h"
#include "heartbeat_hw.h"
#include "imu_app.h"
#include "motor.h"
#include "motor_app.h"
#include "oled_app.h"
#include "zf_common_clock.h"
#include "vision_link.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    grayscale_app_init();
    motor_app_init();
    imu_app_init();
    oled_app_init();
    button_app_init();
    uart3_maix_app_init();
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
    emm42_demo_app_init();
#endif

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
        imu_app_process();
        grayscale_app_process();
        vision_link_process();
        motor_app_process();
        uart3_maix_app_process();
        heartbeat_app_process();
        button_app_process();
        oled_app_process();
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
        emm42_demo_app_process();
#endif
    }
}
