#include <assert.h>
#include <stdio.h>
/* MSVC host test: firmware uses GCC packed attributes. */
#ifdef _MSC_VER
#define __attribute__(x)
#pragma pack(push, 1)
#endif
#include "flight_log.h"
#ifdef _MSC_VER
#pragma pack(pop)
#endif

int main(void)
{
    assert(sizeof(flight_log_record_t) == 40);
    flight_log_metadata_t saved = {0}, decoded;
    saved.version = FLIGHT_LOG_METADATA_VERSION;
    saved.motor_idle_percent = 5.5f;
    saved.reserved = 25;
    assert(flight_log_metadata_decode(&decoded, &saved));
    assert(memcmp(&saved, &decoded, sizeof(saved)) == 0);
    for (unsigned version = 2; version <= 3; ++version) {
        saved.version = version;
        assert(!flight_log_metadata_decode(&decoded, &saved));
    }
    saved.version = 99;
    assert(!flight_log_metadata_decode(&decoded, &saved));
    puts("Metadata v4 decode tests passed");
    return 0;
}
