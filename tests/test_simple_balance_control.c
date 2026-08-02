#include <assert.h>
#include <stdio.h>

#include "sw1_open_loop.h"
#include "v1_center_controller.h"

static const v1_center_config_t v1_config =
{
    {-1.0f, 1.1f, 0.14f},
    {0.8f, -0.6f, 0.08f},
    0.0f, 0.060f, 0.050f, 0.22f, 0.003f,
    0.003f, 0.005f, 0.005f, 0.010f,
    100u, 5000u, 100u, 3u,
};

static const sw1_open_loop_config_t sw1_config =
{
    {{-1.0f, 900u}, {1.1f, 520u}, {0.8f, 820u}, {-0.6f, 1220u}},
    0.0f, 200u, 200u, 50u, 4800u,
};

static void feed(v1_center_controller_t *controller, uint32 now_ms,
                 float position_m, float velocity_mps, uint8 valid)
{
    v1_center_observation_t observation;

    observation.valid = valid;
    observation.new_measurement = valid;
    observation.position_m = position_m;
    observation.velocity_mps = velocity_mps;
    observation.age_ms = valid ? 10u : 101u;
    v1_center_controller_step(controller, &observation, now_ms);
}

static void recover(v1_center_controller_t *controller, float position_m)
{
    feed(controller, 10u, position_m, 0.0f, 1u);
    feed(controller, 20u, position_m, 0.0f, 1u);
    feed(controller, 30u, position_m, 0.0f, 1u);
}

static void test_v1_triangle_and_brake_lock(void)
{
    v1_center_controller_t controller;
    const v1_center_output_t *output;

    v1_center_controller_init(&controller, &v1_config);
    recover(&controller, -0.020f);
    output = v1_center_controller_get_output(&controller);
    assert(V1_CENTER_ACCEL == output->phase);
    assert(1 == output->direction);
    assert(output->target_angle_deg < 0.0f);

    feed(&controller, 50u, -0.012f, 0.030f, 1u);
    assert(V1_CENTER_BRAKE == controller.output.phase);
    assert(controller.output.target_angle_deg > 0.0f);
    feed(&controller, 70u, 0.002f, 0.004f, 1u);
    assert(V1_CENTER_SETTLE == controller.output.phase);
    assert(0.0f == controller.output.target_angle_deg);
}

static void test_v1_trapezoid_and_stale_vision(void)
{
    v1_center_controller_t controller;

    v1_center_controller_init(&controller, &v1_config);
    recover(&controller, -0.070f);
    feed(&controller, 50u, -0.060f, 0.061f, 1u);
    assert(V1_CENTER_CRUISE == controller.output.phase);
    feed(&controller, 70u, -0.050f, 0.049f, 1u);
    assert(V1_CENTER_ACCEL == controller.output.phase);
    feed(&controller, 90u, -0.020f, 0.050f, 1u);
    assert(V1_CENTER_BRAKE == controller.output.phase);
    feed(&controller, 120u, -0.018f, 0.040f, 0u);
    assert(V1_CENTER_WAIT_VISION == controller.output.phase);
    assert(0.0f == controller.output.target_angle_deg);
}

static void test_sw1_absolute_schedule(void)
{
    sw1_open_loop_t sw1;

    sw1_open_loop_init(&sw1, &sw1_config);
    assert(-1.0f == sw1_open_loop_get_start_angle(&sw1));
    sw1_open_loop_start(&sw1, 100u);
    sw1_open_loop_step(&sw1, 1000u);
    assert(SW1_OPEN_LOOP_POS_BRAKE == sw1.output.phase);
    assert(0u != sw1.output.command_due);
    sw1_open_loop_mark_command_applied(&sw1);
    sw1_open_loop_step(&sw1, 1520u);
    assert(SW1_OPEN_LOOP_TURN_DWELL == sw1.output.phase);
    sw1_open_loop_mark_command_applied(&sw1);
    sw1_open_loop_step(&sw1, 1720u);
    assert(SW1_OPEN_LOOP_NEG_ACCEL == sw1.output.phase);
    sw1_open_loop_mark_command_applied(&sw1);
    sw1_open_loop_step(&sw1, 2540u);
    assert(SW1_OPEN_LOOP_NEG_BRAKE == sw1.output.phase);
    sw1_open_loop_mark_command_applied(&sw1);
    sw1_open_loop_step(&sw1, 3760u);
    assert(SW1_OPEN_LOOP_FINAL_SETTLE == sw1.output.phase);
    sw1_open_loop_mark_command_applied(&sw1);
    sw1_open_loop_step(&sw1, 3960u);
    assert(SW1_OPEN_LOOP_COMPLETE == sw1.output.phase);
    assert(3860u == sw1.output.elapsed_ms);
}

static void test_sw1_missed_deadline(void)
{
    sw1_open_loop_t sw1;

    sw1_open_loop_init(&sw1, &sw1_config);
    sw1_open_loop_start(&sw1, 0u);
    sw1_open_loop_step(&sw1, 960u);
    assert(SW1_OPEN_LOOP_FAULT_DEADLINE_MISSED == sw1.output.fault);
}

int main(void)
{
    test_v1_triangle_and_brake_lock();
    test_v1_trapezoid_and_stale_vision();
    test_sw1_absolute_schedule();
    test_sw1_missed_deadline();
    puts("simple balance controller tests passed");
    return 0;
}
