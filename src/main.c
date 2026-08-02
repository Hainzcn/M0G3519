#include "button_app.h"
#include "balance_app.h"
#include "control_config.h"
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

    /* Preserve late startup diagnostics before the 256-byte UART0 queue fills. */
    heartbeat_hw_uart_flush_blocking();
    heartbeat_hw_uart_send_string(
        "[boot-mode] " __DATE__ " " __TIME__);
#if (BALANCE_CONTROL_ENABLE != 0u)
    heartbeat_hw_uart_send_string(" balance=1");
#else
    heartbeat_hw_uart_send_string(" balance=0");
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

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
        imu_app_process();
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
    }
}
