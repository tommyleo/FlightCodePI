#pragma once
#include <stdbool.h>
bool vtx_tramp_init(void);
void vtx_tramp_update(bool armed);
const char *vtx_tramp_status_name(void);
