#include "motor_app.h"
#include "heartbeat.h"
#include "motor.h"

// ===================== 电机转动测试 Demo 配置（按需修改） =====================
#define MOTOR_APP_DEMO_ENABLE               (0)        // 1=接入主循环运行 demo，0=关闭
#define MOTOR_APP_DEMO_HOLD_FORWARD         (1)        // 1=持续满速正转（联调 VM/输出用），0=正反转交替
#define MOTOR_APP_DEMO_DUTY                 (10000)    // PWM 占空比，范围 [0, MOTOR_SPEED_MAX]
#define MOTOR_APP_DEMO_STEP_MS              (2000)     // 正转/反转各持续毫秒数（HOLD_FORWARD=0 时有效）
#define MOTOR_APP_DEMO_LOOP                 (1)        // 1=循环正反转，0=只跑一轮后停止
// ========================================================================

typedef enum
{
    MOTOR_APP_DEMO_PHASE_FORWARD = 0,
    MOTOR_APP_DEMO_PHASE_REVERSE,
    MOTOR_APP_DEMO_PHASE_DONE,
} motor_app_demo_phase_enum;

static motor_app_demo_phase_enum motor_app_demo_phase = MOTOR_APP_DEMO_PHASE_FORWARD;
static uint32 motor_app_demo_phase_start_ms = 0;
static uint8 motor_app_demo_started = 0;

static int32 motor_app_demo_clamp_duty(int32 duty)
{
    if (duty > (int32)MOTOR_SPEED_MAX)
    {
        return (int32)MOTOR_SPEED_MAX;
    }
    if (duty < 0)
    {
        return 0;
    }
    return duty;
}

static void motor_app_demo_begin_phase(int32 left_speed, int32 right_speed)
{
    motor_app_demo_phase_start_ms = heartbeat_get_ms();
    motor_set_speed(left_speed, right_speed);
}

void motor_app_init(void)
{
    motor_init();
}

void motor_app_demo_process(void)
{
#if !MOTOR_APP_DEMO_ENABLE
    (void)motor_app_demo_phase;
    (void)motor_app_demo_phase_start_ms;
    (void)motor_app_demo_started;
    return;
#else
    const int32 demo_duty = motor_app_demo_clamp_duty(MOTOR_APP_DEMO_DUTY);
    uint32 elapsed_ms;

    if (!motor_app_demo_started)
    {
        motor_app_demo_started = 1;
#if MOTOR_APP_DEMO_HOLD_FORWARD
        motor_set_speed(demo_duty, demo_duty);
#else
        motor_app_demo_phase   = MOTOR_APP_DEMO_PHASE_FORWARD;
        motor_app_demo_begin_phase(demo_duty, demo_duty);
#endif
        return;
    }

#if MOTOR_APP_DEMO_HOLD_FORWARD
    return;
#else

    if (MOTOR_APP_DEMO_PHASE_DONE == motor_app_demo_phase)
    {
        return;
    }

    elapsed_ms = heartbeat_get_ms() - motor_app_demo_phase_start_ms;
    if (elapsed_ms < MOTOR_APP_DEMO_STEP_MS)
    {
        return;
    }

    if (MOTOR_APP_DEMO_PHASE_FORWARD == motor_app_demo_phase)
    {
        motor_app_demo_phase = MOTOR_APP_DEMO_PHASE_REVERSE;
        motor_app_demo_begin_phase(-demo_duty, -demo_duty);
        return;
    }

    motor_stop();
#if MOTOR_APP_DEMO_LOOP
    motor_app_demo_phase = MOTOR_APP_DEMO_PHASE_FORWARD;
    motor_app_demo_begin_phase(demo_duty, demo_duty);
#else
    motor_app_demo_phase = MOTOR_APP_DEMO_PHASE_DONE;
#endif
#endif  /* MOTOR_APP_DEMO_HOLD_FORWARD */
#endif  /* MOTOR_APP_DEMO_ENABLE */
}
