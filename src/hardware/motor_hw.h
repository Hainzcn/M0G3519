#ifndef MOTOR_HW_H_
#define MOTOR_HW_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

/*
 * TB6612FNG 双 PWM 驱动（原厂推荐接法）：
 *   正转 IN1=PWM、IN2=低；反转 IN1=低、IN2=PWM；占空比与转速同向。
 *
 * 引脚：左 A0/A1，右 B12/B13，均为 TIM_A0 CH0~CH3，17 kHz。
 * STBY/VCC → 3.3V；VM → 电机电源；GND 共地。
 */

#define MOTOR_HW_PWM_FREQ_HZ    (17000)
#define MOTOR_HW_DUTY_MAX       (PWM_DUTY_MAX)

typedef enum
{
    MOTOR_HW_LEFT = 0,
    MOTOR_HW_RIGHT,
    MOTOR_HW_NUM,
} motor_hw_index_enum;

void motor_hw_init(void);
void motor_hw_set_duty(motor_hw_index_enum motor, int32 duty);
void motor_hw_brake(motor_hw_index_enum motor);
void motor_hw_stop_all(void);

#endif
