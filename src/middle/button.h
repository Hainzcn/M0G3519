#ifndef BUTTON_H_
#define BUTTON_H_

#include "zf_common_typedef.h"

#define BUTTON_DEBOUNCE_MS          (20)

typedef enum
{
    BUTTON_ID_NONE = 0,
    BUTTON_ID_SW1,
    BUTTON_ID_SW2,
    BUTTON_ID_SW3,
    BUTTON_ID_SW4,
} button_id_t;

void        button_init(void);
void        button_process(void);
button_id_t button_get_active(void);
const char *button_get_name(button_id_t id);

#endif
