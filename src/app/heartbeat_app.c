#include "heartbeat_app.h"
#include "heartbeat.h"

void heartbeat_app_init(void)
{
    heartbeat_init();
}

void heartbeat_app_process(void)
{
    heartbeat_process();
}
