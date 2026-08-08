#include "button_app.h"
#include "ab_run_app.h"
#include "buzzer.h"
#include "control_config.h"
#include "balance_simple_app.h"
#include "drive_balance_demo_app.h"
#include "uart3_maix_app.h"
#include "grayscale_app.h"
#include "heartbeat_app.h"
#include "heartbeat_hw.h"
#include "imu_app.h"
#include "motor.h"
#include "motor_app.h"
#include "no_load_lap_app.h"
#include "stop_test_app.h"
#include "oled_app.h"
#include "zf_common_clock.h"
#include "vision_link.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    grayscale_app_init();
    motor_app_init();
    ab_run_app_init();
    no_load_lap_app_init();
    stop_test_app_init();
    imu_app_init();
    oled_app_init();
    buzzer_init();
    button_app_init();
    uart3_maix_app_init();

    /* Preserve late startup diagnostics before the 256-byte UART0 queue fills. */
    heartbeat_hw_uart_flush_blocking();
    heartbeat_hw_uart_send_string(
        "[boot-mode] " __DATE__ " " __TIME__);
#if (UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)
    heartbeat_hw_uart_send_string(" uart3=balance-telemetry\r\n");
#elif (UART3_MAIX_MODE == UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG)
    heartbeat_hw_uart_send_string(" uart3=chassis-telemetry\r\n");
#else
    heartbeat_hw_uart_send_string(" uart3=operational-telemetry\r\n");
#endif
    heartbeat_hw_uart_flush_blocking();

    balance_simple_app_init();
    drive_balance_demo_app_init();

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
        imu_app_process();
        grayscale_app_process();
        vision_link_process();
        motor_app_process();
        no_load_lap_app_process();
        ab_run_app_process();
        drive_balance_demo_app_process();
        balance_simple_app_process();
        stop_test_app_process();
        uart3_maix_app_process();
        heartbeat_app_process();
        button_app_process();
        buzzer_process();
        oled_app_process();
    }
}
