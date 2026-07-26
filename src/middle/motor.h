#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_typedef.h"

#define MOTOR_SPEED_MAX     (10000)        // 归一化速度满量程，与 hardware 层占空比量程对齐

void motor_init(void);
void motor_set_speed(int32 left_speed, int32 right_speed);    // 输入范围 [-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]，当前为开环直通映射
void motor_brake(void);
void motor_stop(void);

/*
 * 预留闭环控制扩展位置：
 * 后续接入编码器（左轮占位引脚 B10/B11/B9，右轮占位引脚 A26/B27/A27，详见 docs/pin/pin.md）后，
 * 可在本层新增例如 motor_set_target_rpm() 的闭环入口，内部维护速度 PID/前馈，
 * 计算结果仍通过 motor_set_speed() 下发到 hardware 层，无需改动 hardware 层接口。
 * 当前阶段仅做开环占空比映射，不依赖编码器硬件。
 */

#endif
