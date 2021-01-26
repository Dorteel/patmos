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


// stores receiver values in an global array
void intr_handler(void) {
    // read the receiver pwm duty cycle
    channel_1 = receiver_read(0);
    channel_2 = receiver_read(1);
    channel_3 = receiver_read(2);
    channel_4 = receiver_read(3);
    channel_5 = receiver_read(4);
    channel_6 = receiver_read(5);
}


void change_settings(void) {
  adjustable_setting_1 = variable_1_to_adjust;
  adjustable_setting_2 = variable_2_to_adjust;
  adjustable_setting_3 = variable_3_to_adjust;

  for (error = 0; error < 150; error ++) {
    millis(20);
    send_telemetry_data();
  }
  error = 0;

  while (channel_6 >= 1900) {
    micros(3700);
    send_telemetry_data();
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
