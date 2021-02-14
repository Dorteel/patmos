#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "receiver/receiver.h"



// Takes 215 us on average
int main(int argc, char **argv)
{
  printf("Receiver check!\n");

  while(1)
  {
    // Start timer
    unsigned int start = (get_cpu_usecs());
    
    // Read values
    int thrust = receive(0);
    int roll = receive(1);
    int pitch = receive(2);
    int yaw = receive(3);
    int channel5 = receive(4);
    int channel6 = receive(5);
    
    // Stop timer
    unsigned now = (get_cpu_usecs());
    printf("Runtime: %d\n",(now-start) );
    
    // Print readings
    printf("Thrust: %d\tRoll: %d\tPitch: \t%d\tYaw: %d\tCh5: %d\tCh6: %d\n",thrust,roll,pitch,yaw,channel5,channel6);

    // Stop condition
    if(thrust < 1050 && yaw > 1950 && roll < 1050 && pitch > 1950)
    {
      break;
    }

  }
  return 0;
}

