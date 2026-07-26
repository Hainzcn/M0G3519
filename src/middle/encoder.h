#ifndef ENCODER_H_
#define ENCODER_H_

#include "zf_common_typedef.h"

/*
 * GM37-520 减速电机 + 11 线增量式编码器，减速比 30:1。
 * 逐飞库 encoder_quad_init() 使用 QEI 正交 4 倍频，输出轴每转计数：
 *   11 线 × 4 × 30 = 1320 counts/rev
 *
 * 硬件 QEI 计数器为 16 位，中间层对原始计数做环形差分并累加到 int32 总里程。
 */
#define ENCODER_MOTOR_MODEL_LINES           (11)
#define ENCODER_MOTOR_GEAR_RATIO            (30)
#define ENCODER_QUAD_EDGE_MULTIPLIER        (4)
#define ENCODER_COUNTS_PER_WHEEL_REV        (ENCODER_MOTOR_MODEL_LINES * ENCODER_QUAD_EDGE_MULTIPLIER * ENCODER_MOTOR_GEAR_RATIO)
#define ENCODER_RAW_COUNT_MOD               (65536U)

void encoder_init(void);

uint16 encoder_get_left_raw_count(void);
uint16 encoder_get_right_raw_count(void);
int32 encoder_get_left_total_count(void);
int32 encoder_get_right_total_count(void);

void encoder_clear_left_count(void);
void encoder_clear_right_count(void);
void encoder_clear_all_count(void);

void encoder_update_speed(uint32 period_ms);
int32 encoder_get_left_rpm(void);
int32 encoder_get_right_rpm(void);

#endif
