#include "vtx_tramp.h"

#include <stdint.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "tramp.pio.h"
#define now_ms() ((uint32_t)(time_us_64()/1000u))
#include "flight_settings.h"

#define FRAME_SIZE 16U
#define BAUD_RATE 9600U
#define START_DELAY_MS 1000U
#define REQUEST_PERIOD_MS 200U
#define RESPONSE_TIMEOUT_MS 300U
#define MAX_RETRIES 20U

typedef enum { IDLE, START_DELAY, WAIT_LIMITS, WAIT_STATUS,
               DELAY_STATUS_QUERY, DONE, FAILED } tramp_state_t;
typedef enum { STATUS_NOT_CONFIGURED, STATUS_INITIALIZING, STATUS_APPLIED,
               STATUS_INVALID_SETTINGS, STATUS_UART_ERROR, STATUS_NO_RESPONSE,
               STATUS_RACE_LOCKED, STATUS_NOT_CONFIRMED } tramp_status_t;
static PIO bus = pio2;
static int sm = -1;
static uint offset, pin;
static uint8_t outgoing[FRAME_SIZE], outgoing_position;
static bool sending;
static void service_tx(void)
{
 while(sending && outgoing_position < FRAME_SIZE && !pio_sm_is_tx_fifo_full(bus,sm))
  pio_sm_put(bus,sm,outgoing[outgoing_position++]);
 if(outgoing_position == FRAME_SIZE) sending=false;
}
static void close_bus(void)
{
 if(sm<0)return;
 pio_sm_set_enabled(bus,sm,false);
 gpio_init(pin);
 pio_sm_unclaim(bus,sm);
 pio_remove_program(bus,&tramp_program,offset);
 sm=-1;
}
static bool open_bus(uint port)
{
 sm=pio_claim_unused_sm(bus,false);
 if(sm<0)return false;
 if(!pio_can_add_program(bus,&tramp_program)){pio_sm_unclaim(bus,sm);sm=-1;return false;}
 offset=pio_add_program(bus,&tramp_program);
 pin=2u*(port-1u);
 pio_gpio_init(bus,pin);
 gpio_pull_up(pin);
 return true;
}
static uint8_t writes;
static uint8_t state;
static uint8_t status_code;
static uint16_t desired_frequency, desired_power;
static uint8_t response[FRAME_SIZE], response_position, retries;
static uint32_t deadline_ms;

static const uint16_t frequencies_eu[5][8] = {
 {5865,5845,5825,5805,5785,5765,5745,0},{5733,5752,5771,5790,5809,5828,5847,5866},
 {0,0,0,0,0,0,0,0},{5740,5760,5780,5800,5820,5840,5860,0},{0,0,5732,5769,5806,5843,0,0}};
static const uint16_t frequencies_us[5][8] = {
 {5865,5845,5825,5805,5785,5765,5745,5725},{5733,5752,5771,5790,5809,5828,5847,5866},
 {5705,5685,5665,0,5885,5905,0,0},{5740,5760,5780,5800,5820,5840,5860,5880},
 {5658,5695,5732,5769,5806,5843,5880,5917}};

const char *vtx_tramp_status_name(void)
{

 switch((tramp_status_t)status_code) {
 case STATUS_INITIALIZING: return "INITIALIZING";
 case STATUS_APPLIED: return "APPLIED";
 case STATUS_INVALID_SETTINGS: return "INVALID_SETTINGS";
 case STATUS_UART_ERROR: return "UART_ERROR";
 case STATUS_NO_RESPONSE: return "NO_RESPONSE";
 case STATUS_NOT_CONFIRMED: return "NOT_CONFIRMED";
 case STATUS_RACE_LOCKED: return "RACE_LOCKED";
 default: return "NOT_CONFIGURED";
 }
}
static uint8_t checksum(const uint8_t frame[FRAME_SIZE])
{ uint8_t result=0; for(uint32_t i=1;i<14;i++) result+=frame[i]; return result; }
static bool valid(uint8_t command)
{ return response[0]==0x0F && response[1]==command && response[14]==checksum(response) && response[15]==0; }
static void fail(tramp_status_t code) { status_code=(uint8_t)code; state=FAILED; }

static bool send_frame(uint8_t command, uint16_t value)
{
 if(sm<0 || sending)return false;
 memset(outgoing,0,sizeof(outgoing)); outgoing[0]=0x0F; outgoing[1]=command;
 outgoing[2]=(uint8_t)value; outgoing[3]=(uint8_t)(value>>8); outgoing[14]=checksum(outgoing);
 pio_sm_set_enabled(bus,sm,false);
 pio_sm_config config=tramp_program_get_default_config(offset);
 sm_config_set_out_pins(&config,pin,1);
 sm_config_set_set_pins(&config,pin,1);
 sm_config_set_in_pins(&config,pin);
 sm_config_set_in_shift(&config,true,false,32);
 sm_config_set_out_shift(&config,true,false,32);
 sm_config_set_clkdiv(&config,(float)clock_get_hz(clk_sys)/(8.0f*BAUD_RATE));
 pio_sm_init(bus,sm,offset,&config);
 pio_sm_set_pins_with_mask(bus,sm,1u<<pin,1u<<pin);
 outgoing_position=0; sending=true;
 service_tx();
 pio_sm_set_enabled(bus,sm,true);
 return true;
}
static bool begin_query(uint8_t command, tramp_state_t waiting)
{
 if(!send_frame(command,0)) return false;
 memset(response,0,sizeof(response)); response_position=0; state=waiting;
 deadline_ms=now_ms()+RESPONSE_TIMEOUT_MS; return true;
}
static bool receive_byte(uint8_t *value)
{
 if(sm<0 || pio_sm_is_rx_fifo_empty(bus,sm))return false;
 *value=(uint8_t)(pio_sm_get(bus,sm)>>24);
 return true;
}
static bool collect_response(void)
{
 uint8_t value; while(receive_byte(&value)) {
  if(response_position==0 && value!=0x0F) continue;
  response[response_position++]=value; if(response_position==FRAME_SIZE) return true;
 } return false;
}
static void schedule_status_query(void)
{ state=DELAY_STATUS_QUERY; deadline_ms=now_ms()+REQUEST_PERIOD_MS; }

bool vtx_tramp_init(void)
{
 const flight_settings_t *s=flight_settings_get();
 close_bus(); sending=false;
 if(s->vtx_protocol!=VTX_PROTOCOL_TRAMP){status_code=STATUS_NOT_CONFIGURED;state=IDLE;return false;}
 if(s->vtx_band>=5||s->vtx_channel>=8){fail(STATUS_INVALID_SETTINGS);return false;}
 desired_frequency=s->vtx_region==1u?frequencies_us[s->vtx_band][s->vtx_channel]:frequencies_eu[s->vtx_band][s->vtx_channel];
 if(!desired_frequency||!s->vtx_power_mw||s->vtx_power_mw>UINT16_MAX){fail(STATUS_INVALID_SETTINGS);return false;}
 desired_power=(uint16_t)s->vtx_power_mw;
 if(s->vtx_uart==s->receiver_uart || !open_bus(s->vtx_uart)){fail(STATUS_UART_ERROR);return false;}
 writes=0; retries=0;status_code=STATUS_INITIALIZING;state=START_DELAY;deadline_ms=now_ms()+START_DELAY_MS;return true;
}
static void retry_query(uint8_t command,tramp_state_t waiting)
{
 if(++retries>=MAX_RETRIES) fail(STATUS_NO_RESPONSE);
 else if(!begin_query(command,waiting)) fail(STATUS_UART_ERROR);
}
void vtx_tramp_update(bool armed)
{
 service_tx();
 if(armed||state==IDLE||state==DONE||state==FAILED)return;
 uint32_t now=now_ms();
 if(state==START_DELAY){if((int32_t)(now-deadline_ms)>=0&&!begin_query('r',WAIT_LIMITS))fail(STATUS_UART_ERROR);return;}
 if(state==DELAY_STATUS_QUERY){if((int32_t)(now-deadline_ms)>=0&&!begin_query('v',WAIT_STATUS))fail(STATUS_UART_ERROR);return;}
 uint8_t expected=state==WAIT_LIMITS?'r':'v';
 if(!collect_response()){if((int32_t)(now-deadline_ms)>=0)retry_query(expected,state);return;}
 if(!valid(expected)){retry_query(expected,state);return;}
 retries=0;
 if(state==WAIT_LIMITS){
  uint16_t minimum=response[2]|((uint16_t)response[3]<<8);
  uint16_t maximum=response[4]|((uint16_t)response[5]<<8);
  uint16_t max_power=response[6]|((uint16_t)response[7]<<8);
  if(!minimum || desired_frequency<minimum || desired_frequency>maximum || desired_power>max_power)
   fail(STATUS_INVALID_SETTINGS);
  else if(!begin_query('v',WAIT_STATUS))fail(STATUS_UART_ERROR);
  return;
 }
 uint16_t frequency=(uint16_t)response[2]|((uint16_t)response[3]<<8);
 uint16_t power=(uint16_t)response[4]|((uint16_t)response[5]<<8);
 if(response[6]&1)fail(STATUS_RACE_LOCKED);
 else if((frequency!=desired_frequency || power!=desired_power) && writes++>=20u)fail(STATUS_NOT_CONFIRMED);
 else if(frequency!=desired_frequency){if(!send_frame('F',desired_frequency))fail(STATUS_UART_ERROR);else schedule_status_query();}
 else if(power!=desired_power){if(!send_frame('P',desired_power))fail(STATUS_UART_ERROR);else schedule_status_query();}
 else{status_code=STATUS_APPLIED;state=DONE;}
}
