#include "motor_hw.h"

typedef struct
{
    pwm_channel_enum in1;
    pwm_channel_enum in2;
} motor_hw_channel_struct;

// 四路 PWM 全部落在 TIM_A0 的 CH0~CH3，freq 只需按最后一次 pwm_init 生效即可保持一致
static const motor_hw_channel_struct motor_hw_channel[MOTOR_HW_NUM] =
{
    [MOTOR_HW_LEFT]  = { PWM_TIM_A0_CH0_A0,  PWM_TIM_A0_CH1_A1  },     // TB6612 AIN1 / AIN2
    [MOTOR_HW_RIGHT] = { PWM_TIM_A0_CH2_B12, PWM_TIM_A0_CH3_B13 },     // TB6612 BIN1 / BIN2
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
    uint8 i;
    for (i = 0; i < MOTOR_HW_NUM; i ++)
    {
        pwm_init(motor_hw_channel[i].in1, MOTOR_HW_PWM_FREQ_HZ, 0);
        pwm_init(motor_hw_channel[i].in2, MOTOR_HW_PWM_FREQ_HZ, 0);
    }
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
