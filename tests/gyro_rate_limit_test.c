#include <assert.h>
#include <stdio.h>
#include "../src/storage/flight_settings.c"

uint8_t test_flash[PICO_FLASH_SIZE_BYTES];
void flash_range_erase(uint32_t offset, size_t count)
{ assert(offset+count<=sizeof(test_flash)); memset(test_flash+offset,255,count); }
void flash_range_program(uint32_t offset,const uint8_t *data,size_t count)
{ assert(offset+count<=sizeof(test_flash)); memcpy(test_flash+offset,data,count); }

int main(void)
{
    memset(test_flash,255,sizeof(test_flash));
    flight_settings_reset_defaults();
    assert(flight_settings_get()->main_loop_hz==16000);
    assert(flight_settings_get()->gyro_rate_hz==8000);
    assert(imu_gyro_rate_supported(8000));
    assert(!imu_gyro_rate_supported(16000));
    flight_settings_t settings=*flight_settings_get();
    settings.roll.kp=123; settings.gyro_lpf_hz=80; settings.dynamic_d_boost_percent=0;
    assert(flight_settings_set(&settings));
    settings.gyro_rate_hz=16000;
    assert(!flight_settings_set(&settings));
    assert(flight_settings_get()->gyro_rate_hz==8000);
    settings_record_t stored={.magic=SETTINGS_MAGIC,.version=SETTINGS_VERSION,.settings=settings};
    stored.checksum=hash_record(&stored,offsetof(settings_record_t,checksum));
    memcpy(test_flash+SETTINGS_FLASH_OFFSET,&stored,sizeof(stored));
    flight_settings_init();
    assert(flight_settings_get()->gyro_rate_hz==8000);
    assert(flight_settings_get()->main_loop_hz==16000);
    assert(flight_settings_get()->roll.kp==123);
    assert(flight_settings_get()->gyro_lpf_hz==80);
    assert(flight_settings_get()->dynamic_d_boost_percent==0);
    assert(!flight_settings_are_saved());
    assert(flight_settings_save());
    flight_settings_init();
    assert(flight_settings_are_saved());
    assert(flight_settings_get()->gyro_rate_hz==8000);
    assert(flight_settings_get()->roll.kp==123);
    settings=*flight_settings_get(); settings.main_loop_hz=8000;
    assert(flight_settings_set(&settings));
    puts("Gyro limit: defaults, request rejection, 16/8 and 8/8, saved-settings normalization and persistence passed");
}
