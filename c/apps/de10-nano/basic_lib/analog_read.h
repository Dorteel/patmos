//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_ANALOG_H
#define PATMOS_ANALOG_H
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>


//battery voltage read register
#define BATTERY ( ( volatile _IODEV unsigned * )  PATMOS_IO_AUDIO )



//battery voltage read function(yet to be received from DTU)
float batteryRead()
{
  return *(BATTERY);
}




#endif //PATMOS_ANALOG_H
