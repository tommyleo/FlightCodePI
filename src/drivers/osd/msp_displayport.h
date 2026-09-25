#pragma once

#include <stdbool.h>
#include <stdint.h>

void msp_displayport_init(void);
void msp_displayport_update(float voltage, bool armed, uint32_t now_us);
void msp_displayport_process(void);
bool msp_displayport_is_available(void);
const char *msp_displayport_status_name(void);
