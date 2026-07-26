/*********************************************************************************************************************
 * Project-local ZF headfile (device layer trimmed for dual-drive base).
 * Based on SeekFree MSPM0G3519 opensource library.
 ********************************************************************************************************************/
#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

#include "stdio.h"
#include "stdint.h"
#include "string.h"

#include "ti_msp_dl_config.h"
#include "zf_sdk_compat.h"

#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"
#include "zf_driver_timer.h"
#include "zf_driver_uart.h"
#include "zf_driver_encoder.h"

#endif
