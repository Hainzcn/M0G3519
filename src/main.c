#include "bluetooth_test_app.h"
#include "heartbeat_app.h"
#include "heartbeat_hw.h"
#include "motor.h"
#include "zf_common_clock.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    heartbeat_app_init();
    motor_init();
    motor_stop();
    bluetooth_test_app_init();

    while (1)
    {
        motor_watchdog_kick();
        heartbeat_hw_uart_tx_pump();
        bluetooth_test_app_process();
        heartbeat_app_process();
    }
}
