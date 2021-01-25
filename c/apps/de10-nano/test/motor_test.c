#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../Flight_conteroller_v2.h"
include "../gps/read_gps.h"
// #include "../gyro/gyro.h"
// #include "../barometer/baro.h"
// #include "../compass/compass.h"
#include "../basic_lib/actuator_receiver.h"
// #include "../pid/pid.h"
// #include "../safety/return_to_home.h"
// #include "../safety/led_signal.h"
// #include "../safety/start_stop_takeoff.h"
// #include "../callibration/callibration.h"
// #include "../basic_lib/analog_read.h"


int main(int argc, char **argv)
{
  printf("motors check!\n");

  for (int i = 0; i < 2; ++i)
  {

    actuator_write(m1, 1100);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);

    millis(2000);

    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1100);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);

    millis(2000);

    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1100);
    actuator_write(m4, 1000);

    millis(2000);

    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1100);

    millis(2000);

    actuator_write(m1, 1100);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1100);
    actuator_write(m3, 1100);
    actuator_write(m4, 1100);

    millis(2000);                                //Set the timer for the next loop.
  
  }


  return 0;
}

