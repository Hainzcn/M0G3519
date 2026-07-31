#ifndef MOTOR_HW_H_
#define MOTOR_HW_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

/*
 * TB6612FNG 标准控制接口：
 *   PWMA/PWMB 是独立 PWM 输入；AIN1/AIN2、BIN1/BIN2 仅为方向输入。
 *   正转：IN1=高、IN2=低、PWM=占空比；反转时两个方向电平互换。
 *   PWM=低时输出高阻滑行；PWM=高且两个方向输入相同时为短路刹车。
 *
 * 接线：
 *   PWMA=A0 (TIM_A0 CH0)，PWMB=A1 (TIM_A0 CH1)。
 *   AIN1=B2，AIN2=B3，BIN1=B4，BIN2=B5。
 *   B2~B5 为连续 GPIO 区域；避开 A30/A31、B0/B1 按键及 QEI 引脚。
 *   STBY/VCC → 3.3V；VM → 5~12V 电机电源；GND 共地。
 */

#define MOTOR_HW_PWM_FREQ_HZ    (17000)
#define MOTOR_HW_DUTY_MAX       (PWM_DUTY_MAX)

#define MOTOR_HW_LEFT_PWMA_PWM           (PWM_TIM_A0_CH0_A0)
#define MOTOR_HW_LEFT_AIN1_GPIO          (B2)
#define MOTOR_HW_LEFT_AIN2_GPIO          (B3)

#define MOTOR_HW_RIGHT_PWMB_PWM          (PWM_TIM_A0_CH1_A1)
#define MOTOR_HW_RIGHT_BIN1_GPIO         (B4)
#define MOTOR_HW_RIGHT_BIN2_GPIO         (B5)

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
