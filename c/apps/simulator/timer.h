//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_TIMER_H
#define PATMOS_TIMER_H
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>

//delay function ins microseconds
void micros(int microseconds)
{
    unsigned int timer_ms = (get_cpu_usecs());
    unsigned int loop_timer = timer_ms;
    while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
}

void millis(int milliseconds)
{
    unsigned int timer_ms = (get_cpu_usecs()/1000);
    unsigned int loop_timer = timer_ms;
    while(timer_ms - loop_timer < milliseconds)timer_ms = (get_cpu_usecs()/1000);
}


#endif //PATMOS_TIMER_H
