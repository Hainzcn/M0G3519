/*
 * Lightweight replacement for the SeekFree library debug subsystem.
 *
 * The MSPM0G3519_Library drivers call debug_assert_handler() for parameter
 * validation. The original implementation pulls in the whole debug UART /
 * FIFO / printf stack (~6.8 KB + C library formatting), which this project
 * never uses.  This stub satisfies the linker while keeping the firmware
 * small; a failed assertion is treated as a no-op in the production build.
 */
#include "zf_common_debug.h"

void debug_assert_handler(uint8 pass, char *file, int line)
{
    (void)file;
    (void)line;
    if (0u == pass)
    {
        /* Assertion failure - intentionally ignored in this build. */
    }
}
