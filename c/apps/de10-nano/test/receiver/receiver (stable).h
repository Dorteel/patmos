//
// Created by rahul on 11/8/20.
//

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

__int32_t receiver_input[6];
///(th,roll,pitch,yaw)
int low[4]={1003,1004,1000,999};
int center[4]={1493,1503,1496,1331};
int high[4]={2002,2000,1993,1994};

int channel_1;
int channel_2;
int channel_3;
int channel_4;
int channel_5;
int channel_6;

int loop_timer;

int program_off;
//get pulse width data from receiver
int receiver_read(unsigned int receiver_id){

    unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
    unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

    return pulse_high_time;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
int convert_receiver_channel(unsigned int function)
{
  unsigned int  channel;       //First we declare some local variables
  int reverse = 1;            //Reverse = -1 when the transmitter channels are reversed, else 1
  int actual;                           ///(0,th,roll,pitch,yaw)
  int difference;

  if (function == 0) channel = 1;//roll
  else if (function == 1){
    reverse = -1;
    channel = 2;//pitch
  }
  else if (function == 2) channel = 0;//throttle
  else channel = 3;//yaw

  actual = receiver_input[channel];      //Read the actual receiver value for the corresponding function

  if (actual <= center[channel]){                                                         //The actual receiver value is lower than the center value
    if (actual < low[channel]) actual = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value                              //If the channel is reversed
    return 1500 - reverse * difference;                                             //If the channel is not reversed
  }
  else{                                                                        //The actual receiver value is higher than the center value
    if (actual > high[channel])actual = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value                            //If the channel is reversed
    return 1500 + reverse * difference;                                             
  }
}

// stores receiver values in an global array
void receive(void) {
    // read the receiver pwm duty cycle

    receiver_input[0] = receiver_read(0);
    receiver_input[1] = receiver_read(1);
    receiver_input[2] = receiver_read(2);
    receiver_input[3] = receiver_read(3);

    channel_1 = convert_receiver_channel(0);  //1(0)               //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    channel_2 = convert_receiver_channel(1);  //2(1)               //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    channel_3 = convert_receiver_channel(2);  //0(0)               //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    channel_4 = convert_receiver_channel(3);  //3(0)               //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    channel_5 = receiver_read(5);
    channel_6 = receiver_read(4);
}

#endif //PATMOS_ACTUATOR_RECEIVER_H
