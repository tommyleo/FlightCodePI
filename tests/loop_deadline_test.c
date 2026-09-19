#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "loop_deadline.h"

static uint64_t simulate(int drift, uint32_t start, uint32_t *missed)
{
    const uint32_t period=6000; /* 96 MHz / 16 kHz */
    uint64_t elapsed=0;
    uint32_t deadline=start;
    for(unsigned i=0;i<16000;++i){
        deadline+=period;
        elapsed+=i%16==0?9200:3000; /* A short service overrun each millisecond. */
        uint32_t now=start+(uint32_t)elapsed;
        if(drift){if((int32_t)(now-deadline)>=0)deadline=now;}
        else deadline=loop_deadline_recover(deadline,now,period,missed);
        int32_t wait=(int32_t)(deadline-now);
        if(wait>0)elapsed+=(uint32_t)wait;
    }
    return elapsed;
}
int main(void)
{
    uint32_t missed=0;
    assert(loop_deadline_recover(1000,900,100,&missed)==1000);
    assert(loop_deadline_recover(1000,1099,100,&missed)==1000);
    assert(missed==0);
    assert(loop_deadline_recover(1000,1450,100,&missed)==1400);
    assert(missed==4);
    assert(loop_deadline_recover(20,UINT32_MAX-20,100,&missed)==20);
    assert(loop_deadline_recover(UINT32_MAX-20,20,100,&missed)==UINT32_MAX-20);
    missed=0;
    uint64_t old=simulate(1,0,&missed),fixed=simulate(0,0,&missed);
    assert(fixed==96000000 && missed==0 && old>fixed);
    assert(simulate(0,UINT32_MAX-3000,&missed)==fixed && missed==0);
    printf("16 kHz simulation: previous %.2f Hz; fixed %.2f Hz; missed slots %u\n",
           16000.0*96000000.0/(double)old,16000.0*96000000.0/(double)fixed,missed);
    for(uint32_t hz=8000;hz<=16000;hz+=8000){
        uint32_t remainder=0,total=0;
        for(unsigned i=0;i<hz;++i){uint32_t step=loop_period_advance(hz,1,&remainder);assert(step==125||step==62||step==63);total+=step;}
        assert(total==1000000 && remainder==0);
        uint32_t a=0,b=0,whole=loop_period_advance(hz,123,&a),parts=0;
        for(unsigned i=0;i<123;++i)parts+=loop_period_advance(hz,1,&b);
        assert(whole==parts && a==b);
    }
    /* Sustained overload remains visible, with missed slots counted. */
    missed=0;uint32_t deadline=0,now=0;
    for(unsigned i=0;i<1000;++i){deadline+=6000;now+=9000;deadline=loop_deadline_recover(deadline,now,6000,&missed);}
    assert(missed>0 && deadline<=now && now-deadline<6000);
    puts("Absolute phase, fractional periods, sustained overload and timer wrap passed");
}
