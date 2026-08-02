#ifndef BALANCE_CONFIG_H_
#define BALANCE_CONFIG_H_

#include "lever_actuator.h"
#include "sw1_open_loop.h"
#include "v1_center_controller.h"

typedef struct
{
    float soft_edge_position_m;
    float hard_edge_position_m;
    float edge_progress_m;
    uint32 edge_progress_timeout_ms;
    uint8 min_vision_confidence;
    uint32 vision_transport_latency_ms;
    uint32 vision_max_compensation_ms;
} balance_safety_config_t;

extern const lever_actuator_config_t balance_lever_config;
extern const v1_center_config_t balance_v1_config;
extern const sw1_open_loop_config_t balance_sw1_config;
extern const balance_safety_config_t balance_safety_config;

#endif
