#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "telemetry_text.h"
int main(void)
{
    char buffer[384];telemetry_text_t text={buffer,sizeof(buffer),0,true};
    telemetry_literal(&text,"@CFG TELEMETRY");telemetry_uint(&text,UINT32_MAX);
    telemetry_fixed(&text,16000.0f,1);telemetry_fixed(&text,-123.456f,3);
    telemetry_fixed(&text,.005f,2);telemetry_fixed(&text,0,3);telemetry_char(&text,'\n');
    assert(text.valid && strcmp(buffer,"@CFG TELEMETRY 4294967295 16000.0 -123.456 0.01 0.000\n")==0);
    for(unsigned decimals=0;decimals<=3;++decimals){
        float unit=1;for(unsigned j=0;j<decimals;++j)unit*=.1f;
        for(int i=-20000;i<=20000;++i){
            float value=i*.1234567f;text=(telemetry_text_t){buffer,sizeof(buffer),0,true};
            telemetry_fixed(&text,value,decimals);assert(text.valid);
            assert(fabsf(strtof(buffer,NULL)-value)<=unit*.501f+.0005f);
        }
    }
    char tiny[5]={'!','!','!','!','#'};text=(telemetry_text_t){tiny,4,0,true};
    telemetry_literal(&text,"too long");assert(!text.valid && tiny[3]=='\0' && tiny[4]=='#');
    text=(telemetry_text_t){buffer,sizeof(buffer),0,true};telemetry_fixed(&text,NAN,3);assert(!text.valid);
    text=(telemetry_text_t){buffer,sizeof(buffer),0,true};telemetry_fixed(&text,INFINITY,3);assert(!text.valid);
    puts("Telemetry field format, 160004 rounding cases, bounds and non-finite values passed");
}
