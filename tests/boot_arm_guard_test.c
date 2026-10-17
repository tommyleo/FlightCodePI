#include <assert.h>
#include "../src/app/boot_arm_guard.h"
int main(void) {
 boot_arm_guard_t g={0};
 boot_arm_guard_update(&g,false,false); assert(!g.checked && !g.blocked);
 boot_arm_guard_update(&g,false,true); assert(!g.checked && !g.blocked);
 boot_arm_guard_update(&g,true,true); assert(g.checked && g.blocked);
 boot_arm_guard_update(&g,true,false); assert(g.blocked);
 boot_arm_guard_update(&g,true,true); assert(g.blocked);
 g=(boot_arm_guard_t){0};
 boot_arm_guard_update(&g,true,false); assert(g.checked && !g.blocked);
 boot_arm_guard_update(&g,true,true); assert(!g.blocked);
 boot_arm_guard_update(&g,false,true); assert(!g.blocked);
 return 0;
}
