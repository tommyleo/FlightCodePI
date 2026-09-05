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
    flight_log_metadata_t saved = {0}, decoded;
    saved.version = 3;
    saved.motor_idle_percent = 5.5f;
    saved.reserved = 25;
    const float values[] = {0,10,100,300,1000};
    for (unsigned i=0;i<5;i++) {
        saved.throttle_rise_ms=values[i];
        assert(flight_log_metadata_decode(&decoded,&saved));
        assert(decoded.throttle_rise_ms==values[i]);
        assert(decoded.motor_idle_percent==5.5f && decoded.reserved==25);
    }
    /* Legacy data has only 128 bytes, no ramp or extension padding. */
    unsigned char legacy[128];
    saved.version=2;
    memcpy(legacy,&saved,sizeof(legacy));
    memset(&decoded,0xff,sizeof(decoded));
    assert(flight_log_metadata_decode(&decoded,legacy));
    assert(decoded.version==2 && decoded.throttle_rise_ms==-1.0f);
    assert(decoded.motor_idle_percent==5.5f && decoded.reserved==25);
    saved.version=99;
    assert(!flight_log_metadata_decode(&decoded,&saved));
    puts("Metadata v3 and legacy v2 decode tests passed");
    return 0;
}
