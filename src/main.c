#include "button_app.h"
#include "balance_app.h"
#include "control_config.h"
#if (BALL_RETURN_DEMO_ENABLE != 0u)
#include "ball_return_demo_app.h"
#endif
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
#include "emm42_demo_app.h"
#endif
#include "uart3_maix_app.h"
#include "grayscale_app.h"
#include "heartbeat_app.h"
#include "heartbeat_hw.h"
#if (EMM42_BALANCE_DEMO_ENABLE == 0u)
#include "imu_app.h"
#endif
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
#if (EMM42_BALANCE_DEMO_ENABLE == 0u)
    imu_app_init();
#endif
    oled_app_init();
    button_app_init();
    uart3_maix_app_init();

    /* Preserve late startup diagnostics before the 256-byte UART0 queue fills. */
    heartbeat_hw_uart_flush_blocking();
    heartbeat_hw_uart_send_string(
        "[boot-mode] " __DATE__ " " __TIME__);
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
    heartbeat_hw_uart_send_string(" demo=1");
#else
    heartbeat_hw_uart_send_string(" demo=0");
#endif
#if (BALL_RETURN_DEMO_ENABLE != 0u)
    heartbeat_hw_uart_send_string(" ball-return=1");
#else
    heartbeat_hw_uart_send_string(" ball-return=0");
#endif
#if (BALANCE_CONTROL_ENABLE != 0u)
    heartbeat_hw_uart_send_string(" balance=1");
#else
    heartbeat_hw_uart_send_string(" balance=0");
#endif
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
    heartbeat_hw_uart_send_string(" imu=0");
#else
    heartbeat_hw_uart_send_string(" imu=1");
#endif
#if (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)
    heartbeat_hw_uart_send_string(" uart3=balance-telemetry\r\n");
#elif (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
    heartbeat_hw_uart_send_string(" uart3=chassis-telemetry\r\n");
#else
    heartbeat_hw_uart_send_string(" uart3=normal\r\n");
#endif
    heartbeat_hw_uart_flush_blocking();
#if (BALANCE_CONTROL_ENABLE != 0u)
    balance_app_init();
#endif
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
    emm42_demo_app_init();
    heartbeat_hw_uart_flush_blocking();
#endif
#if (BALL_RETURN_DEMO_ENABLE != 0u)
    ball_return_demo_app_init();
    heartbeat_hw_uart_flush_blocking();
#endif

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
#if (EMM42_BALANCE_DEMO_ENABLE == 0u)
        imu_app_process();
#endif
        grayscale_app_process();
        vision_link_process();
        motor_app_process();
#if (BALANCE_CONTROL_ENABLE != 0u)
        balance_app_process();
#endif
        uart3_maix_app_process();
        heartbeat_app_process();
        button_app_process();
        oled_app_process();
#if (EMM42_BALANCE_DEMO_ENABLE != 0u)
        emm42_demo_app_process();
#endif
#if (BALL_RETURN_DEMO_ENABLE != 0u)
        ball_return_demo_app_process();
#endif
    }
}
