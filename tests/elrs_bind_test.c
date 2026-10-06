#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "crsf_bind.h"

typedef struct { uint32_t receiver_protocol; } flight_settings_t;
#define RECEIVER_PROTOCOL_CRSF 1U
static flight_settings_t settings;
static bool armed, motor_test_enabled, pid_simulation_enabled, tx_success;
static unsigned tx_count;
static const char *response;
static const flight_settings_t *flight_settings_get(void) { return &settings; }
static bool flight_control_is_armed(void) { return armed; }
static bool sbus_bind_receiver(void) { ++tx_count; return tx_success; }
static bool sbus_receiver_bind(void) { ++tx_count; return tx_success; }
static void reply(const char *s) { response = s; }
#define printf(s) reply(s)
static void command(const char *command)
{
#include "elrs_bind_command_under_test.inc"
}
#undef printf

static uint8_t crc8(const uint8_t *p, unsigned n, uint8_t polynomial)
{
    uint8_t crc = 0;
    while (n--) {
        crc ^= *p++;
        for (unsigned i=0; i<8; ++i)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? polynomial : 0));
    }
    return crc;
}

int main(void)
{
    assert(sizeof(crsf_bind_frame) == 9);
    assert(crsf_bind_frame[0] == 0xc8 && crsf_bind_frame[1] == 7);
    assert(crsf_bind_frame[2] == 0x32 && crsf_bind_frame[3] == 0xec);
    assert(crsf_bind_frame[4] == 0xc8 && crsf_bind_frame[5] == 0x10);
    assert(crsf_bind_frame[6] == 1);
    assert(crc8(crsf_bind_frame+2, 5, 0xba) == crsf_bind_frame[7]);
    assert(crc8(crsf_bind_frame+2, 6, 0xd5) == crsf_bind_frame[8]);
    settings.receiver_protocol=1; tx_success=true; armed=true;
    command("BIND_RECEIVER"); assert(strcmp(response,"@CFG ERROR ARMED\n")==0 && tx_count==0);
    armed=false; motor_test_enabled=true;
    command("BIND_RECEIVER"); assert(strcmp(response,"@CFG ERROR BIND_BUSY\n")==0 && tx_count==0);
    motor_test_enabled=false; pid_simulation_enabled=true;
    command("BIND_RECEIVER"); assert(strcmp(response,"@CFG ERROR BIND_BUSY\n")==0 && tx_count==0);
    pid_simulation_enabled=false; settings.receiver_protocol=0;
    command("BIND_RECEIVER"); assert(strcmp(response,"@CFG ERROR BIND_REQUIRES_ELRS\n")==0 && tx_count==0);
    settings.receiver_protocol=1; tx_success=false;
    command("BIND_RECEIVER"); assert(strcmp(response,"@CFG ERROR BIND_TX_FAILED\n")==0 && tx_count==1);
    tx_success=true;
    command("BIND_RECEIVER"); assert(strcmp(response,"@CFG OK BIND_RECEIVER\n")==0 && tx_count==2);
    response=NULL; command("UNKNOWN"); assert(response==NULL && tx_count==2);
    puts("CRSF command/frame CRC and firmware binding guards: PASS");
}
