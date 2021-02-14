#ifndef PATMOS_ACTUATOR_RECEIVER_H
#define PATMOS_ACTUATOR_RECEIVER_H
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//Receiver controller register
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

///(th,roll,pitch,yaw)
int low[4]={1003,1004,1000,999};
int center[4]={1493,1503,1496,1331};
int high[4]={2002,2000,1993,1994};

/*
 * Function: receiver_read
 * ----------------------------
 * Description:
 *     Reads the register
 *
 * Arguments:
 *    receiver_id: 0,1,2,3 for thrust, roll, pitch, yaw respectively
 *
 * Returns: pulse_high_time: the reading
 */
int receiver_read(unsigned int receiver_id){

    unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
    unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

    return pulse_high_time;
}

/*
 * Function: receive
 * ----------------------------
 * Description:
 *     Given the channel name, outputs a value in a 1000 - 2000 range
 *
 * Arguments:
 *    channel:
 *      0 : thrust
 *      1 : roll
 *      2 : pitch (is inverted)
 *      3 : yaw
 *      4 : ?
 *      5 : ?      
 *
 * Returns: None
 */
int receive(int channel) {
  int reverse = 1;
  int reading = receiver_read(channel);
  int difference = 0;

  // Channels other than RPY + Thrust (no conversion needed)
  if (channel >= 4) return reading;

  // Pitch is reversed
  if (channel == 2) reverse = -1;

  if (reading <= center[channel]){                                                         //The actual receiver value is lower than the center value
    if (reading < low[channel]) reading = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = -((long)(center[channel] - reading) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value                              //If the channel is reversed
  }
  else{                                                                        //The actual receiver value is higher than the center value
    if (reading > high[channel])reading = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(reading - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value                            //If the channel is reversed                                            
  }
  return 1500 + reverse * difference;   
}

#endif //PATMOS_ACTUATOR_RECEIVER_H
