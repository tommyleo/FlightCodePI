#pragma once
#include <stdbool.h>

/* Only the first valid receiver frame determines the boot interlock.
 * Once blocked, only a processor restart may clear it. */
typedef struct { bool checked; bool blocked; } boot_arm_guard_t;
static inline void boot_arm_guard_update(boot_arm_guard_t *guard,
                                         bool valid, bool arm_active)
{
    if (!guard->checked && valid) {
        guard->checked = true;
        guard->blocked = arm_active;
    }
}
