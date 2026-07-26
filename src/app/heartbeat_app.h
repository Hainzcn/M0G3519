#ifndef HEARTBEAT_APP_H_
#define HEARTBEAT_APP_H_

void heartbeat_app_init(void);     // 启动状态指示灯与心跳串口报文（PA14 LED，UART0 PA10/PA11）
void heartbeat_app_process(void);  // 在 main 主循环中调用，处理待发送的心跳

#endif
