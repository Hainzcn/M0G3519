#ifndef MOTOR_HW_H_
#define MOTOR_HW_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

/*
 * TB6612FNG 双电机、双 PWM 驱动模式（AIN1/AIN2、BIN1/BIN2 均输出 PWM）。
 * STBY 由硬件电路直接拉高，本层不控制 STBY 引脚。
 *
 * 引脚分配（四路均落在 TIM_A0，共享同一频率寄存器，占空比寄存器各自独立）：
 *   左电机 AIN1 -> A0   (PWM_TIM_A0_CH0_A0)
 *   左电机 AIN2 -> A1   (PWM_TIM_A0_CH1_A1)
 *   右电机 BIN1 -> B12  (PWM_TIM_A0_CH2_B12)
 *   右电机 BIN2 -> B13  (PWM_TIM_A0_CH3_B13)
 *
 * 单路控制逻辑：正转时 IN1 输出 PWM、IN2 输出低；反转相反；占空比为 0 时两路皆低（滑行停止）。
 */

#define MOTOR_HW_PWM_FREQ_HZ    (17000)            // TB6612 PWM 频率，需按实际电机噪声/效率验证
#define MOTOR_HW_DUTY_MAX       (PWM_DUTY_MAX)     // 占空比满量程，对应 zf_driver_pwm 的 0~10000

typedef enum
{
    MOTOR_HW_LEFT = 0,
    MOTOR_HW_RIGHT,
    MOTOR_HW_NUM,
} motor_hw_index_enum;

void motor_hw_init(void);
void motor_hw_set_duty(motor_hw_index_enum motor, int32 duty);        // duty 范围 [-MOTOR_HW_DUTY_MAX, MOTOR_HW_DUTY_MAX]，符号决定转向，0 为滑行停止
void motor_hw_brake(motor_hw_index_enum motor);                       // 双臂同时拉满，短路刹车
void motor_hw_stop_all(void);

#endif
