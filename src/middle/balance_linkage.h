#ifndef BALANCE_LINKAGE_H_
#define BALANCE_LINKAGE_H_

#include "zf_common_typedef.h"

uint8 balance_linkage_motor_from_physical_lever_deg(float lever_angle_deg,
                                                     float *motor_angle_deg);
uint8 balance_linkage_physical_lever_from_motor_deg(float motor_angle_deg,
                                                     float *lever_angle_deg);

#endif
