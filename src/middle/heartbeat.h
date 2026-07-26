#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#include "zf_common_typedef.h"

#define HEARTBEAT_PERIOD_MS     (1000)     // 心跳周期 1Hz（LED 翻转 + 串口报文）

void heartbeat_init(void);
void heartbeat_process(void);              // 在 main 主循环中反复调用，不可在中断服务函数中调用
uint32 heartbeat_get_ms(void);             // 自 SysTick 启动以来的毫秒计数

#endif
