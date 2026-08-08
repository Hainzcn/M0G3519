#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "balance_actuator_trajectory.h"

int main(void)
{
    balance_actuator_trajectory_config_t config = {4.0f, 30.0f, 600.0f};
    balance_actuator_trajectory_t trajectory;
    float previous_angle = 0.0f;
    float previous_rate = 0.0f;
    uint8 saw_saturation = 0u;

    balance_actuator_trajectory_init(&trajectory, &config);
    for (uint32 step = 0u; step < 100u; step++)
    {
        balance_actuator_trajectory_step(&trajectory, 8.0f, 0.02f);
        assert(fabsf(trajectory.output.rate_deg_s) <= 30.0001f);
        assert(fabsf(trajectory.output.rate_deg_s - previous_rate) <= 12.0001f);
        assert(trajectory.output.angle_deg >= previous_angle - 0.0001f);
        assert(trajectory.output.angle_deg <= 4.0001f);
        if (trajectory.output.saturated) saw_saturation = 1u;
        previous_angle = trajectory.output.angle_deg;
        previous_rate = trajectory.output.rate_deg_s;
    }
    if (fabsf(trajectory.output.angle_deg - 4.0f) >= 0.0001f)
        fprintf(stderr, "final angle %.6f rate %.6f\n",
                trajectory.output.angle_deg, trajectory.output.rate_deg_s);
    assert(fabsf(trajectory.output.angle_deg - 4.0f) < 0.0001f);
    assert(saw_saturation != 0u);

    for (uint32 step = 0u; step < 100u; step++)
        balance_actuator_trajectory_step(&trajectory, -4.0f, 0.02f);
    assert(fabsf(trajectory.output.angle_deg + 4.0f) < 0.0001f);
    assert(fabsf(trajectory.output.rate_deg_s) < 0.0001f);
    puts("balance actuator trajectory tests passed");
    return 0;
}
