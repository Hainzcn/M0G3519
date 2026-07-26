#include "heartbeat_app.h"
#include "motor_app.h"
#include "zf_common_clock.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);       // 含 SYSCFG_DL_init（含 UART0 PA10/PA11 初始化）

    heartbeat_app_init();              // 先启动串口心跳，便于联调
    motor_app_init();

    while (1)
    {
        heartbeat_app_process();
    }
}
