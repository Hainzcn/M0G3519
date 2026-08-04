#ifndef LINE_CONTROL_H_
#define LINE_CONTROL_H_

#include "grayscale.h"
#include "zf_common_typedef.h"

typedef enum
{
    LINE_TRACK_PHASE_STRAIGHT = 0,
    LINE_TRACK_PHASE_RIGHT_ARC,
} line_track_phase_t;

typedef struct
{
    float left_rpm;
    float right_rpm;
    float error;
    float pid_turn_rpm;
    float curvature_feedforward_rpm;
    float turn_rpm;
    float lookup_correction_rpm;
    float phase_distance_m;
    float curve_blend;
    float speed_scale;
    float feedback_scale;
    uint8 active_count;
    uint8 right_active_count;
    uint8 right_curve_detected;
    uint8 sensor_mask;
    int8 lookup_level;
    line_track_phase_t phase;
    uint8 line_valid;
    uint8 marker_detected;
    uint8 wide_pattern_filtered;
    uint8 line_lost;
} line_control_output_t;

void line_control_init(void);
void line_control_reset(void);
void line_control_set_base_rpm(float base_rpm);
void line_control_set_base_rpm_immediate(float base_rpm);
float line_control_get_base_rpm(void);
float line_control_get_base_accel_rpm_s(void);
float line_control_get_base_accel_preview_rpm_s(float preview_s);
void line_control_update(const uint8 values[GRAYSCALE_CHANNELS],
                         uint32 now_ms, float dt_s);
const line_control_output_t *line_control_get_output(void);

#endif
