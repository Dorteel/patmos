//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_ACTUATOR_RECEIVER_H
#define PATMOS_ACTUATOR_RECEIVER_H
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>
#include "timer.h"


const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

//Receiver controller register
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )


//writes pwm signlas of width=data to the esc
void actuator_write(unsigned int actuator_id, unsigned int data)
{
    *(MOTOR + actuator_id) = data;
}

//get pulse width data from receiver
int receiver_read(unsigned int receiver_id){

    unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
    unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

    return pulse_high_time;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
int convert_receiver_channel(unsigned int function)
{
  unsigned int  channel, reverse;       //First we declare some local variables
  int actual;                           ///(0,th,roll,pitch,yaw)
  int difference;

  if(function==0)
  {
    reverse = 0;                        //Reverse =1 when the transmitter channels are reversed, else 0
    channel =1;//roll
  }
  else if(function==1)
  {
    reverse = 1;
    channel=2;//pitch
  }
  else if(function==2)
  {
    reverse = 0;
    channel=0;//throttle
  }
  else
  {
    reverse = 0;                                            
    channel =3;//yaw
  }

  actual = receiver_input[channel];      //Read the actual receiver value for the corresponding function
  // low = 1000;  //Store the low value for the specific receiver input channel
  // center = 1500; //Store the center value for the specific receiver input channel
  // high = 2000;   //Store the high value for the specific receiver input channel

  if(actual < center[channel]){                                                         //The actual receiver value is lower than the center value
    if(actual < low[channel])actual = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center[channel]){                                                                        //The actual receiver value is higher than the center value
    if(actual > high[channel])actual = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

// stores receiver values in an global array
void intr_handler(void) {
    // read the receiver pwm duty cycle
  while(!program_off)
  {
      
      receiver_input[0] = receiver_read(0);
      receiver_input[1] = receiver_read(1);
      receiver_input[2] = receiver_read(2);
      receiver_input[3] = receiver_read(3);

      int channel_1_tmp = convert_receiver_channel(0);  //1(0)               //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
      int channel_2_tmp = convert_receiver_channel(1);  //2(1)               //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
      int channel_3_tmp = convert_receiver_channel(2);  //0(0)               //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
      int channel_4_tmp = convert_receiver_channel(3);  //3(0)               //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
      int channel_5_tmp = receiver_read(5);
      int channel_6_tmp = receiver_read(4);
      pthread_mutex_lock(&mutex);
      channel_1 = channel_1_tmp;
      channel_2 = channel_2_tmp;
      channel_3 = channel_3_tmp;
      channel_4 = channel_4_tmp;
      channel_5 = channel_5_tmp;
      channel_6 = channel_6_tmp;
      pthread_mutex_unlock(&mutex);

      if(motor_publish)
      {
        actuator_write(m1, esc_1);                                                 //give motors 1000us pulse.
        actuator_write(m2, esc_2);
        actuator_write(m3, esc_3);
        actuator_write(m4, esc_4);
        motor_publish=0;
      }
  }
  if(PRINT_COMMANDS)printf("intr_end");
}


void change_settings(void) {
  adjustable_setting_1 = variable_1_to_adjust;
  adjustable_setting_2 = variable_2_to_adjust;
  adjustable_setting_3 = variable_3_to_adjust;

  // for (error = 0; error < 150; error ++) {
    // send_telemetry_data();
  // }
  error = 0;
  pthread_mutex_lock(&mutex);
  pthread_mutex_unlock(&mutex);
  while (channel_6 >= 1900) {
    // send_telemetry_data();
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
    if (channel_1 > 1550)adjustable_setting_1 += (float)(channel_1 - 1550) * 0.000001;
    if (channel_1 < 1450)adjustable_setting_1 -= (float)(1450 - channel_1) * 0.000001;
    if (adjustable_setting_1 < 0)adjustable_setting_1 = 0;
    variable_1_to_adjust = adjustable_setting_1;
    
    if (channel_2 > 1550)adjustable_setting_2 += (float)(channel_2 - 1550) * 0.000001;
    if (channel_2 < 1450)adjustable_setting_2 -= (float)(1450 - channel_2) * 0.000001;
    if (adjustable_setting_2 < 0)adjustable_setting_2 = 0;
    variable_2_to_adjust = adjustable_setting_2;

    if (channel_4 > 1550)adjustable_setting_3 += (float)(channel_4 - 1550) * 0.000001;
    if (channel_4 < 1450)adjustable_setting_3 -= (float)(1450 - channel_4) * 0.000001;
    if (adjustable_setting_3 < 0)adjustable_setting_3 = 0;
    variable_3_to_adjust = adjustable_setting_3;
  }
  loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
}





#endif //PATMOS_ACTUATOR_RECEIVER_H
