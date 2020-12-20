//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_LED_H
#define PATMOS_LED_H
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>


//LEDs register
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )

//Blinks the LEDs once
void blink_once()
{
    int i, j;
    for (i=2000; i!=0; --i)
        for (j=2000; j!=0; --j)
            LED = 0x0001;
    for (i=2000; i!=0; --i)
        for (j=2000; j!=0; --j)
            LED = 0x0000;
    return;
}

///1 to switch on LED and 0 to off
void LED_out(int i){
    if(i==1) LED = 0x0001;
    else LED = 0x0000;
    return;
}

#endif //PATMOS_LED_H
