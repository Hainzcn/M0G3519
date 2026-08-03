#include "balance_linkage.h"

#include <math.h>

/* Physical protractor angle to signed Emm42 absolute position. */
#define BALANCE_LINKAGE_FIT_QUADRATIC   (-0.154232927f)
#define BALANCE_LINKAGE_FIT_LINEAR      (-3.641589613f)
#define BALANCE_LINKAGE_FIT_OFFSET      (-18.922007248f)
#define BALANCE_LINKAGE_MIN_LEVER_DEG   (-7.05f)
#define BALANCE_LINKAGE_MAX_LEVER_DEG   (2.76f)

uint8 balance_linkage_motor_from_physical_lever_deg(float lever_angle_deg,
                                                     float *motor_angle_deg)
{
    if ((NULL == motor_angle_deg) ||
        (lever_angle_deg < BALANCE_LINKAGE_MIN_LEVER_DEG) ||
        (lever_angle_deg > BALANCE_LINKAGE_MAX_LEVER_DEG))
    {
        return 0u;
    }

    *motor_angle_deg =
        BALANCE_LINKAGE_FIT_QUADRATIC * lever_angle_deg * lever_angle_deg +
        BALANCE_LINKAGE_FIT_LINEAR * lever_angle_deg +
        BALANCE_LINKAGE_FIT_OFFSET;
    return 1u;
}

uint8 balance_linkage_physical_lever_from_motor_deg(float motor_angle_deg,
                                                     float *lever_angle_deg)
{
    float discriminant;
    float physical_angle_deg;

    if (NULL == lever_angle_deg)
    {
        return 0u;
    }

    discriminant = BALANCE_LINKAGE_FIT_LINEAR * BALANCE_LINKAGE_FIT_LINEAR -
                   4.0f * BALANCE_LINKAGE_FIT_QUADRATIC *
                       (BALANCE_LINKAGE_FIT_OFFSET - motor_angle_deg);
    if (discriminant < 0.0f)
    {
        return 0u;
    }

    /* Select the root that is monotonic over the measured linkage travel. */
    physical_angle_deg =
        (-BALANCE_LINKAGE_FIT_LINEAR - sqrtf(discriminant)) /
        (2.0f * BALANCE_LINKAGE_FIT_QUADRATIC);
    if ((physical_angle_deg < BALANCE_LINKAGE_MIN_LEVER_DEG) ||
        (physical_angle_deg > BALANCE_LINKAGE_MAX_LEVER_DEG))
    {
        return 0u;
    }

    *lever_angle_deg = physical_angle_deg;
    return 1u;
}
