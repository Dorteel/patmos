#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../Flight_controller_v2.h"
#include "../gps/read_gps.h"
#include "../telemetry/send_telemetry_data.h"
#include "../basic_lib/actuator_receiver.h"
#include "../gyro/gyro.h"
#include "../compass/compass.h"
#include "../callibration/callibration.h"


int main(int argc, char **argv)
{
  printf("receiver check!\n");

  loop_timer = get_cpu_usecs(); 
  while(1)
  {
    unsigned int start = (get_cpu_usecs());
    intr_handler(); 
    unsigned now = (get_cpu_usecs());
    printf("Runtime: %d\n",(now-start) );
    //printf("channel1: %d, channel2: %d, channel3: %d, channel4: %d,channel5: %d,channel6: %d\n",channel_1,channel_2,channel_3,channel_4,channel_5,channel_6 );

     //For starting the motors: throttle low and yaw left (step 1).
    if(channel_3 < 1050 && channel_4 < 1050)
    {
      start = 1;
      printf("motors arming\n");
    }
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && channel_3 < 1050 && channel_4 > 1450){
      start = 2;
      printf("motors start\n");
    }
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && channel_3 < 1050 && channel_4 > 1950)
    {
      start = 0;
      printf("motors stop\n");
    }


    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      program_off = -1;
      printf("code stop\n");
    }

    if(receiver_watchdog < 750)receiver_watchdog ++;
    if(receiver_watchdog == 750 && start == 2){
      channel_1 = 1500;
      channel_2 = 1500;
      channel_3 = 1500;
      channel_4 = 1500;
      printf("channels to mid position\n");
      error = 8;
      if (number_used_sats > 5){
        if(home_point_recorded == 1)channel_5 = 2000;
        else channel_5 = 1750;
      }
      else channel_5 = 1500;    
    }
    //Some functions are only accessible when the quadcopter is off.
    if (start == 0) {
      //For compass calibration move both sticks to the top right.
       if (channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900)
        {
            printf("callibrating compass....\n");
            callibrate_compass();
            printf("compass callibration complete\n");
        }
      //Level calibration move both sticks to the top left.
      if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)
        {
            printf("level callibration start\n");
            callibrate_level();
            printf("level callibration complete\n");
        }
    }
    

    while (get_cpu_usecs() - loop_timer < 20000);                                            //We wait until 4000us are passed.

    if(program_off == -1)
    {
        break;
    }

  }
  return 0;
}

