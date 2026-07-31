#include "emm42_demo_app.h"

#include <math.h>

#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"

#define EMM42_DEMO_ADDRESS             (EMM42_DEFAULT_ADDRESS)
#define EMM42_DEMO_RPM                 (30u)
#define EMM42_DEMO_ACCELERATION        (20u)
#define EMM42_DEMO_POWER_WAIT_MS       (3000u)
#define EMM42_DEMO_COMMAND_WAIT_MS     (100u)
#define EMM42_DEMO_ENDPOINT_WAIT_MS    (1500u)

#define EMM42_DEMO_ALPHA_DEG           (5.0f)
#define EMM42_DEMO_CB_CM               (21.0f)
#define EMM42_DEMO_DX_CM               (15.5f)
#define EMM42_DEMO_DY_CM               (-0.5f)
#define EMM42_DEMO_DP_CM               (3.5f)
#define EMM42_DEMO_BP_CM               (4.6f)
#define EMM42_DEMO_LINKAGE_BRANCH      (-1.0f)
#define EMM42_DEMO_PI                  (3.14159265358979323846f)
#define EMM42_DEMO_RAD_TO_DEG          (180.0f / EMM42_DEMO_PI)
#define EMM42_DEMO_DEG_TO_RAD          (EMM42_DEMO_PI / 180.0f)

static emm42_demo_state_enum emm42_demo_state;
static uint32 emm42_demo_state_start_ms;
static float emm42_demo_positive_motor_deg;
static float emm42_demo_negative_motor_deg;
static float emm42_demo_target_angle_deg;

static uint8 emm42_demo_elapsed(uint32 now_ms, uint32 wait_ms)
{
    return ((now_ms - emm42_demo_state_start_ms) >= wait_ms) ? 1u : 0u;
}

static void emm42_demo_set_state(emm42_demo_state_enum state, uint32 now_ms)
{
    emm42_demo_state = state;
    emm42_demo_state_start_ms = now_ms;
}

static void emm42_demo_fail(uint32 now_ms, const char *message)
{
    (void)emm42_stop(EMM42_DEMO_ADDRESS, 0u);
    heartbeat_hw_uart_send_string(message);
    emm42_demo_set_state(EMM42_DEMO_ERROR, now_ms);
}

static uint8 emm42_demo_linkage_inverse(float alpha_rad, float *theta_rad)
{
    float db_x;
    float db_y;
    float rho_sq;
    float rho;
    float acos_arg;
    float gamma;
    float eta;

    if (NULL == theta_rad)
    {
        return 0u;
    }

    db_x = EMM42_DEMO_CB_CM * cosf(alpha_rad) - EMM42_DEMO_DX_CM;
    db_y = EMM42_DEMO_CB_CM * sinf(alpha_rad) - EMM42_DEMO_DY_CM;
    rho_sq = db_x * db_x + db_y * db_y;
    rho = sqrtf(rho_sq);

    if ((rho < fabsf(EMM42_DEMO_DP_CM - EMM42_DEMO_BP_CM)) ||
        (rho > (EMM42_DEMO_DP_CM + EMM42_DEMO_BP_CM)))
    {
        return 0u;
    }

    acos_arg = (EMM42_DEMO_DP_CM * EMM42_DEMO_DP_CM + rho_sq -
                EMM42_DEMO_BP_CM * EMM42_DEMO_BP_CM) /
               (2.0f * EMM42_DEMO_DP_CM * rho);
    if (acos_arg > 1.0f)
    {
        acos_arg = 1.0f;
    }
    else if (acos_arg < -1.0f)
    {
        acos_arg = -1.0f;
    }

    gamma = atan2f(db_y, db_x);
    eta = acosf(acos_arg);
    *theta_rad = gamma + EMM42_DEMO_LINKAGE_BRANCH * eta;
    return 1u;
}

static uint8 emm42_demo_calculate_targets(void)
{
    float theta_zero;
    float theta_positive;
    float theta_negative;
    float alpha = EMM42_DEMO_ALPHA_DEG * EMM42_DEMO_DEG_TO_RAD;

    if ((0u == emm42_demo_linkage_inverse(0.0f, &theta_zero)) ||
        (0u == emm42_demo_linkage_inverse(alpha, &theta_positive)) ||
        (0u == emm42_demo_linkage_inverse(-alpha, &theta_negative)))
    {
        return 0u;
    }

    emm42_demo_positive_motor_deg =
        (theta_positive - theta_zero) * EMM42_DEMO_RAD_TO_DEG;
    emm42_demo_negative_motor_deg =
        (theta_negative - theta_zero) * EMM42_DEMO_RAD_TO_DEG;
    return 1u;
}

static uint8 emm42_demo_move_to(float motor_angle_deg)
{
    return emm42_move_angle(EMM42_DEMO_ADDRESS, motor_angle_deg,
                            EMM42_DEMO_RPM, EMM42_DEMO_ACCELERATION,
                            EMM42_POSITION_ABSOLUTE, 0u);
}

void emm42_demo_app_init(void)
{
    emm42_demo_state_start_ms = heartbeat_get_ms();
    emm42_demo_state = EMM42_DEMO_WAIT_POWER;
    emm42_demo_target_angle_deg = 0.0f;

    if (0u == emm42_demo_calculate_targets())
    {
        heartbeat_hw_uart_send_string(
            "[balance-demo] linkage target unreachable\r\n");
        emm42_demo_state = EMM42_DEMO_ERROR;
        return;
    }

    emm42_init();
    heartbeat_hw_uart_send_string(
        "[balance-demo] HOLD LEVER LEVEL; start in 3 seconds\r\n");
}

void emm42_demo_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    switch (emm42_demo_state)
    {
        case EMM42_DEMO_WAIT_POWER:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_POWER_WAIT_MS))
            {
                if (0u == emm42_set_current_position_zero(EMM42_DEMO_ADDRESS))
                {
                    emm42_demo_fail(now_ms,
                        "[balance-demo] zero command failed\r\n");
                    break;
                }
                heartbeat_hw_uart_send_string(
                    "[balance-demo] horizontal position set to zero\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ZERO, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ZERO:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                if (0u == emm42_set_enabled(EMM42_DEMO_ADDRESS, 1u, 0u))
                {
                    emm42_demo_fail(now_ms,
                        "[balance-demo] enable command failed\r\n");
                    break;
                }
                heartbeat_hw_uart_send_string("[balance-demo] enabled\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ENABLE, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ENABLE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_POSITIVE, now_ms);
            }
            break;

        case EMM42_DEMO_MOVE_POSITIVE:
            if (0u == emm42_demo_move_to(emm42_demo_positive_motor_deg))
            {
                emm42_demo_fail(now_ms,
                    "[balance-demo] +5 deg command failed\r\n");
                break;
            }
            emm42_demo_target_angle_deg = EMM42_DEMO_ALPHA_DEG;
            heartbeat_hw_uart_send_string("[balance-demo] lever -> +5 deg\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_POSITIVE, now_ms);
            break;

        case EMM42_DEMO_WAIT_POSITIVE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_ENDPOINT_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_NEGATIVE, now_ms);
            }
            break;

        case EMM42_DEMO_MOVE_NEGATIVE:
            if (0u == emm42_demo_move_to(emm42_demo_negative_motor_deg))
            {
                emm42_demo_fail(now_ms,
                    "[balance-demo] -5 deg command failed\r\n");
                break;
            }
            emm42_demo_target_angle_deg = -EMM42_DEMO_ALPHA_DEG;
            heartbeat_hw_uart_send_string("[balance-demo] lever -> -5 deg\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_NEGATIVE, now_ms);
            break;

        case EMM42_DEMO_WAIT_NEGATIVE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_ENDPOINT_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_POSITIVE, now_ms);
            }
            break;

        case EMM42_DEMO_ERROR:
        default:
            break;
    }
}

emm42_demo_state_enum emm42_demo_app_get_state(void)
{
    return emm42_demo_state;
}

float emm42_demo_app_get_target_angle_deg(void)
{
    return emm42_demo_target_angle_deg;
}

uint8 emm42_demo_app_is_active(void)
{
    return (EMM42_DEMO_ERROR != emm42_demo_state) ? 1u : 0u;
}
