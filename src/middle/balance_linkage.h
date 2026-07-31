#ifndef BALANCE_LINKAGE_H_
#define BALANCE_LINKAGE_H_

#include "zf_common_typedef.h"

uint8 balance_linkage_inverse_deg(float lever_angle_deg,
                                  float *motor_angle_deg);
uint8 balance_linkage_relative_motor_deg(float reference_lever_angle_deg,
                                         float target_lever_angle_deg,
                                         float *relative_motor_deg);

#endif
