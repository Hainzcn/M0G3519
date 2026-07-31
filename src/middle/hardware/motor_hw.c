#include "motor_hw.h"

typedef struct
{
    pwm_channel_enum pwm;
    gpio_pin_enum    in1;
    gpio_pin_enum    in2;
} motor_hw_tb6612_channel_t;

static const motor_hw_tb6612_channel_t motor_hw_channel[MOTOR_HW_NUM] =
{
    [MOTOR_HW_LEFT]  =
    {
        .pwm = MOTOR_HW_LEFT_PWMA_PWM,
        .in1 = MOTOR_HW_LEFT_AIN1_GPIO,
        .in2 = MOTOR_HW_LEFT_AIN2_GPIO,
    },
    [MOTOR_HW_RIGHT] =
    {
        .pwm = MOTOR_HW_RIGHT_PWMB_PWM,
        .in1 = MOTOR_HW_RIGHT_BIN1_GPIO,
        .in2 = MOTOR_HW_RIGHT_BIN2_GPIO,
    },
};

static int32 motor_hw_clamp(int32 duty)
{
    if (duty > (int32)MOTOR_HW_DUTY_MAX)
    {
        return (int32)MOTOR_HW_DUTY_MAX;
    }
    if (duty < -(int32)MOTOR_HW_DUTY_MAX)
    {
        duty = -(int32)MOTOR_HW_DUTY_MAX;
    }
    return duty;
}

static void motor_hw_set_direction(const motor_hw_tb6612_channel_t *ch, uint8 in1, uint8 in2)
{
    gpio_set_level(ch->in1, in1);
    gpio_set_level(ch->in2, in2);
}

void motor_hw_init(void)
{
    uint8 i;

    for (i = 0; i < MOTOR_HW_NUM; i ++)
    {
        const motor_hw_tb6612_channel_t *ch = &motor_hw_channel[i];

        gpio_init(ch->in1, GPO, GPIO_LOW, GPO_PUSH_PULL);
        gpio_init(ch->in2, GPO, GPIO_LOW, GPO_PUSH_PULL);
        pwm_init(ch->pwm, MOTOR_HW_PWM_FREQ_HZ, 0);
    }
}

void motor_hw_set_duty(motor_hw_index_enum motor, int32 duty)
{
    const motor_hw_tb6612_channel_t *ch;

    if (MOTOR_HW_NUM <= motor)
    {
        return;
    }

    duty = motor_hw_clamp(duty);
    ch   = &motor_hw_channel[motor];

    /* 先关闭 PWM，再切换方向，避免换向瞬间驱动电机。 */
    pwm_set_duty(ch->pwm, 0);

    if (0 < duty)
    {
        motor_hw_set_direction(ch, GPIO_HIGH, GPIO_LOW);
        pwm_set_duty(ch->pwm, (uint32)duty);
    }
    else if (0 > duty)
    {
        motor_hw_set_direction(ch, GPIO_LOW, GPIO_HIGH);
        pwm_set_duty(ch->pwm, (uint32)(-duty));
    }
    else
    {
        motor_hw_set_direction(ch, GPIO_LOW, GPIO_LOW);
    }
}

void motor_hw_brake(motor_hw_index_enum motor)
{
    const motor_hw_tb6612_channel_t *ch;

    if (MOTOR_HW_NUM <= motor)
    {
        return;
    }

    ch = &motor_hw_channel[motor];
    motor_hw_set_direction(ch, GPIO_LOW, GPIO_LOW);
    pwm_set_duty(ch->pwm, MOTOR_HW_DUTY_MAX);
}

void motor_hw_stop_all(void)
{
    uint8 i;

    for (i = 0; i < MOTOR_HW_NUM; i ++)
    {
        motor_hw_set_duty((motor_hw_index_enum)i, 0);
    }
}
