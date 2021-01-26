#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../Flight_controller_v2.h"
#include "../gps/read_gps.h"
#include "../basic_lib/actuator_receiver.h"
#include "../basic_lib/timer.h"


int main(int argc, char **argv)
{
  printf("motors check!\n");

  for (int i = 0; i < 2; ++i)
  {

    printf("motor 1 spinning\n");
    actuator_write(m1, 1100);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);

    millis(2000);

    printf("motor 2 spinning\n");
    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1100);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);

    millis(2000);

    printf("motor 3 spinning\n");
    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1100);
    actuator_write(m4, 1000);

    millis(2000);

    printf("motor 4 spinning\n");
    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1100);

    millis(2000);

    printf("all motors spinning\n");
    actuator_write(m1, 1100);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1100);
    actuator_write(m3, 1100);
    actuator_write(m4, 1100);

    millis(2000);                                //Set the timer for the next loop.
  
  }
    printf("motors stop\n");
    actuator_write(m1, 1000);                                                  //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);


  return 0;
}

