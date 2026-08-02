#include "balance_config.h"

#include "control_config.h"

const lever_actuator_config_t balance_lever_config =
{
    BALANCE_STARTUP_CALIBRATED,
    BALANCE_STARTUP_LEVER_ANGLE_DEG,
    0.0f,
    4.0f,
    -1,
    -1,
    30u,
    20u,
    3000u,
    1500u,
    25u,
    2500u,
    200u,
    100u,
    1000u,
    0.1f,
    1.0f,
    5.0f,
    3u,
};

const v1_center_config_t balance_v1_config =
{
    {-1.0f, 1.1f, 0.14f},
    {0.8f, -0.6f, 0.08f},
    0.0f,
    0.060f,
    0.050f,
    0.22f,
    0.003f,
    0.003f,
    0.005f,
    0.005f,
    0.010f,
    100u,
    5000u,
    100u,
    3u,
};

const sw1_open_loop_config_t balance_sw1_config =
{
    {
        {-1.0f, 900u},
        {1.1f, 520u},
        {0.8f, 820u},
        {-0.6f, 1220u},
    },
    0.0f,
    200u,
    200u,
    50u,
    4800u,
};

const balance_safety_config_t balance_safety_config =
{
    0.100f,
    0.120f,
    0.005f,
    500u,
    50u,
    3u,
    30u,
};
