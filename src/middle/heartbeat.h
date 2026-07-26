#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#define HEARTBEAT_PERIOD_MS     (500)      // 心跳周期（LED 翻转 + 串口报文），可按需调整

void heartbeat_init(void);
void heartbeat_process(void);              // 在 main 主循环中反复调用，不可在中断服务函数中调用

#endif
