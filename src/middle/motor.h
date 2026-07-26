#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_typedef.h"

#define MOTOR_SPEED_MAX     (10000)        // 归一化速度满量程，与 hardware 层占空比量程对齐

void motor_init(void);
void motor_set_speed(int32 left_speed, int32 right_speed);    // 输入范围 [-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]，当前为开环直通映射
void motor_brake(void);
void motor_stop(void);

/*
 * 编码器已接入（正交 QEI，见 encoder.c / docs/pin/pin.md）：
 * 左轮 TIMG8（B10/B11），右轮 TIMG9（B7/B9）。
 * 后续可在本层新增 motor_set_target_rpm() 等闭环入口，读取 encoder_get_*_count()，
 * 计算结果仍通过 motor_set_speed() 下发到 hardware 层。
 * 当前 motor_set_speed() 仍为开环直通映射。
 */

#endif
