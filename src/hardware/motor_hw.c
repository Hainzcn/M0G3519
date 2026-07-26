#include "motor_hw.h"

typedef struct
{
    pwm_channel_enum in1;
    pwm_channel_enum in2;
} motor_hw_channel_struct;

// 四路 PWM 落在 TIM_A0 CH0~CH3；正转 IN1=PWM/IN2=低，反转相反（TB6612 标准双 PWM 接法）
static const motor_hw_channel_struct motor_hw_channel[MOTOR_HW_NUM] =
{
    [MOTOR_HW_LEFT]  = { PWM_TIM_A0_CH0_A0,  PWM_TIM_A0_CH1_A1  },
    [MOTOR_HW_RIGHT] = { PWM_TIM_A0_CH2_B12, PWM_TIM_A0_CH3_B13 },
};

static int32 motor_hw_clamp(int32 duty)
{
    if (duty > (int32)MOTOR_HW_DUTY_MAX)
    {
        duty = (int32)MOTOR_HW_DUTY_MAX;
    }
    else if (duty < -(int32)MOTOR_HW_DUTY_MAX)
    {
        duty = -(int32)MOTOR_HW_DUTY_MAX;
    }
    return duty;
}

void motor_hw_init(void)
{
    /*
     * 同一定时器须按 CH0→CH3 顺序初始化；逐飞库 pwm_init 对 CH2/CH3 须写 CCCTL_23（已修复）。
     */
    pwm_init(motor_hw_channel[MOTOR_HW_LEFT].in1,  MOTOR_HW_PWM_FREQ_HZ, 0);
    pwm_init(motor_hw_channel[MOTOR_HW_LEFT].in2,  MOTOR_HW_PWM_FREQ_HZ, 0);
    pwm_init(motor_hw_channel[MOTOR_HW_RIGHT].in1, MOTOR_HW_PWM_FREQ_HZ, 0);
    pwm_init(motor_hw_channel[MOTOR_HW_RIGHT].in2, MOTOR_HW_PWM_FREQ_HZ, 0);
}

void motor_hw_set_duty(motor_hw_index_enum motor, int32 duty)
{
    if (MOTOR_HW_NUM <= motor)
    {
        return;
    }

    duty = motor_hw_clamp(duty);

    if (0 <= duty)
    {
        pwm_set_duty(motor_hw_channel[motor].in1, (uint32)duty);
        pwm_set_duty(motor_hw_channel[motor].in2, 0);
    }
    else
    {
        pwm_set_duty(motor_hw_channel[motor].in1, 0);
        pwm_set_duty(motor_hw_channel[motor].in2, (uint32)(-duty));
    }
}

void motor_hw_brake(motor_hw_index_enum motor)
{
    if (MOTOR_HW_NUM <= motor)
    {
        return;
    }

    pwm_set_duty(motor_hw_channel[motor].in1, MOTOR_HW_DUTY_MAX);
    pwm_set_duty(motor_hw_channel[motor].in2, MOTOR_HW_DUTY_MAX);
}

void motor_hw_stop_all(void)
{
    uint8 i;
    for (i = 0; i < MOTOR_HW_NUM; i ++)
    {
        motor_hw_set_duty((motor_hw_index_enum)i, 0);
    }
}
