#include <assert.h>
#include <stdio.h>
#include "../src/storage/flight_log.c"
uint8_t test_flash[PICO_FLASH_SIZE_BYTES];
static uint32_t clock_us;
static flight_settings_t settings;
uint32_t time_us_32(void) { return clock_us; }
uint32_t imu_get_gyro_rate_hz(void) { return 16000U; }
const flight_settings_t *flight_settings_get(void) { return &settings; }
void flash_range_erase(uint32_t offset, size_t count) {
    assert(offset+count <= sizeof(test_flash)); memset(test_flash+offset,255,count);
}
void flash_range_program(uint32_t offset, const uint8_t *data, size_t count) {
    assert(offset+count <= sizeof(test_flash)); memcpy(test_flash+offset,data,count);
}
static void sample(void) {
    const float raw[3]={100,-200,300}, gyro[3]={10,-20,30};
    const float sp[3]={1,2,3}, p[3]={5,6,7}, i[3]={0,0,0};
    const float du[3]={100.25f,-120.5f,0}, df[3]={10.01f,-20.02f,0};
    const float ff[3]={0,0,0}, pid[3]={15,-14,7}, motors[4]={10,20,30,40};
    flight_log_record(raw,gyro,sp,p,i,du,df,ff,motors,20,false,63,64,pid);
}
int main(void) {
    memset(test_flash,255,sizeof(test_flash)); settings.main_loop_hz=16000;
    clock_us=UINT32_MAX-1000U; flight_log_init(); flight_log_start(); sample();
    assert(record_count==1); assert(records[0].gyro_raw[0]==1000);
    assert(records[0].gyro[0]==100); assert(records[0].d_unfiltered[0]==10025);
    assert(records[0].d_filtered[0]==1001); assert(records[0].pid[0]==30);
    const float zero[3]={0}, extreme[3]={400,0,0};
    assert(signal_clipped(zero,zero,extreme,zero,zero,zero,zero,zero));
    assert(!signal_clipped(zero,zero,zero,zero,zero,zero,zero,zero));
    clock_us+=1000; sample(); assert(record_count==1);
    clock_us+=1000; sample(); assert(record_count==2);
    clock_us+=7000; sample(); assert(record_count==3);
    clock_us+=100; sample(); assert(record_count==3); /* no invented catch-up samples */
    for (uint32_t n=0;n<FLIGHT_LOG_CAPACITY+10;n++) { clock_us+=2000; sample(); }
    assert(record_count==FLIGHT_LOG_CAPACITY); assert(overwritten_records==13);
    clock_us+=123; flight_log_stop(FLIGHT_LOG_FLAG_STOP_DISARM);
    flight_log_record_t last; assert(flight_log_get(record_count-1,&last));
    assert(last.timestamp_us==clock_us); assert(last.flags&FLIGHT_LOG_FLAG_STOP_DISARM);
    assert(last.overwritten_records==14); assert(last.motor[0]==0);
    clock_us+=LOG_PERSIST_DELAY_US; flight_log_persist_if_ready(); assert(using_flash);
    flight_log_init(); assert(using_flash); assert(record_count==FLIGHT_LOG_CAPACITY);
    assert(flight_log_get(record_count-1,&last)); assert(last.overwritten_records==14);
    flight_log_metadata_t md; assert(flight_log_get_metadata(&md));
    assert(md.version==6); assert(md.log_rate_hz==500);
    ((log_header_t *)(test_flash+LOG_FLASH_OFFSET))->version--;
    flight_log_init(); assert(!using_flash); assert(record_count==0);
    puts("Internal logger lifecycle, quantization, rollover, ring, persistence and old-format rejection passed");
}
