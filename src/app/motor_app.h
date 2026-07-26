#ifndef MOTOR_APP_H_
#define MOTOR_APP_H_

void motor_app_init(void);     // 上电初始化，完成后两侧电机保持停止，避免误转
void motor_app_demo(void);     // 简单自检：正转 -> 反转 -> 停止，默认不在 main 中调用，联调时手动触发验证接线方向

#endif
