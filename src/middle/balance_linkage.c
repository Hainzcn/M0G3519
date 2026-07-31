#include "balance_linkage.h"

#include <math.h>

#define BALANCE_LINKAGE_CB_CM          (21.0f)
#define BALANCE_LINKAGE_DX_CM          (15.5f)
#define BALANCE_LINKAGE_DY_CM          (-0.5f)
#define BALANCE_LINKAGE_DP_CM          (3.5f)
#define BALANCE_LINKAGE_BP_CM          (4.6f)
#define BALANCE_LINKAGE_BRANCH         (-1.0f)
#define BALANCE_LINKAGE_PI             (3.14159265358979323846f)
#define BALANCE_LINKAGE_DEG_TO_RAD     (BALANCE_LINKAGE_PI / 180.0f)
#define BALANCE_LINKAGE_RAD_TO_DEG     (180.0f / BALANCE_LINKAGE_PI)
#define BALANCE_LINKAGE_FIT_QUADRATIC  (0.095f)
#define BALANCE_LINKAGE_FIT_LINEAR     (4.0f)
#define BALANCE_LINKAGE_FIT_OFFSET     (-51.0f)
#define BALANCE_LINKAGE_FIT_MIN_DEG    (-10.0f)
#define BALANCE_LINKAGE_FIT_MAX_DEG    (10.0f)

static float balance_linkage_fitted_motor_deg(float lever_angle_deg)
{
    return BALANCE_LINKAGE_FIT_QUADRATIC * lever_angle_deg *
               lever_angle_deg +
           BALANCE_LINKAGE_FIT_LINEAR * lever_angle_deg +
           BALANCE_LINKAGE_FIT_OFFSET;
}

uint8 balance_linkage_inverse_deg(float lever_angle_deg,
                                  float *motor_angle_deg)
{
    float alpha;
    float db_x;
    float db_y;
    float rho_sq;
    float rho;
    float acos_arg;
    float gamma;
    float eta;

    if (NULL == motor_angle_deg)
    {
        return 0u;
    }

    alpha = lever_angle_deg * BALANCE_LINKAGE_DEG_TO_RAD;
    db_x = BALANCE_LINKAGE_CB_CM * cosf(alpha) - BALANCE_LINKAGE_DX_CM;
    db_y = BALANCE_LINKAGE_CB_CM * sinf(alpha) - BALANCE_LINKAGE_DY_CM;
    rho_sq = db_x * db_x + db_y * db_y;
    rho = sqrtf(rho_sq);
    if ((rho < fabsf(BALANCE_LINKAGE_DP_CM - BALANCE_LINKAGE_BP_CM)) ||
        (rho > (BALANCE_LINKAGE_DP_CM + BALANCE_LINKAGE_BP_CM)))
    {
        return 0u;
    }

    acos_arg = (BALANCE_LINKAGE_DP_CM * BALANCE_LINKAGE_DP_CM + rho_sq -
                BALANCE_LINKAGE_BP_CM * BALANCE_LINKAGE_BP_CM) /
               (2.0f * BALANCE_LINKAGE_DP_CM * rho);
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
    *motor_angle_deg =
        (gamma + BALANCE_LINKAGE_BRANCH * eta) * BALANCE_LINKAGE_RAD_TO_DEG;
    return 1u;
}

uint8 balance_linkage_relative_motor_deg(float reference_lever_angle_deg,
                                         float target_lever_angle_deg,
                                         float *relative_motor_deg)
{
    float reference_motor_deg;
    float target_motor_deg;

    if ((NULL == relative_motor_deg) ||
        (0u == balance_linkage_inverse_deg(reference_lever_angle_deg,
                                           &reference_motor_deg)) ||
        (0u == balance_linkage_inverse_deg(target_lever_angle_deg,
                                           &target_motor_deg)))
    {
        return 0u;
    }
    *relative_motor_deg = target_motor_deg - reference_motor_deg;
    return 1u;
}

uint8 balance_linkage_lever_from_relative_motor_deg(
    float reference_lever_angle_deg,
    float relative_motor_deg,
    float *lever_angle_deg)
{
    float motor_angle_deg;
    float discriminant;
    float lever_deg;

    if ((NULL == lever_angle_deg) ||
        (reference_lever_angle_deg < BALANCE_LINKAGE_FIT_MIN_DEG) ||
        (reference_lever_angle_deg > BALANCE_LINKAGE_FIT_MAX_DEG))
    {
        return 0u;
    }

    motor_angle_deg =
        balance_linkage_fitted_motor_deg(reference_lever_angle_deg) +
        relative_motor_deg;
    discriminant = BALANCE_LINKAGE_FIT_LINEAR *
                       BALANCE_LINKAGE_FIT_LINEAR -
                   4.0f * BALANCE_LINKAGE_FIT_QUADRATIC *
                       (BALANCE_LINKAGE_FIT_OFFSET - motor_angle_deg);
    if (discriminant < 0.0f)
    {
        return 0u;
    }

    /* Select the quadratic root that is continuous around the level point. */
    lever_deg = (-BALANCE_LINKAGE_FIT_LINEAR + sqrtf(discriminant)) /
                (2.0f * BALANCE_LINKAGE_FIT_QUADRATIC);
    if ((lever_deg < BALANCE_LINKAGE_FIT_MIN_DEG) ||
        (lever_deg > BALANCE_LINKAGE_FIT_MAX_DEG))
    {
        return 0u;
    }
    *lever_angle_deg = lever_deg;
    return 1u;
}
