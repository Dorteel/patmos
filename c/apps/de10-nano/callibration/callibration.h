#ifndef PATMOS_CALLIB_H
#define PATMOS_CALLIB_H

#include "../basic_lib/i2c_master.h"
#include "../basic_lib/timer.h"
#include "../basic_lib/led.h"



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the level and compass calibration procedres are handled.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void callibrate_compass(void) {
  printf("compass callibration\n");
   compass_calibration_on = 1;                                                //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
   LED_out(1);                                                             //The red led will indicate that the compass calibration is active.
   LED_out(0);                                                            //Turn off the green led as we don't need it.
   while (channel_2 < 1900) {                                                 //Stay in this loop until the pilot lowers the pitch stick of the transmitter.
    intr_handler();
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
     // micros(3700);                                                 //Simulate a 250Hz program loop.
     read_compass();                                                          //Read the raw compass values.
     //In the following lines the maximum and minimum compass values are detected and stored.
     if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
     if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
     if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
     if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
     if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
     if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
   }
   compass_calibration_on = 0;                                                //Reset the compass_calibration_on variable.


   setup_compass();                                                           //Initiallize the compass and set the correct registers.
   read_compass();                                                            //Read and calculate the compass data.
   angle_yaw = actual_compass_heading;                                        //Set the initial compass heading.

   LED_out(0);
   for (error = 0; error < 15; error ++) {
     LED_out(1);
     millis(50);
     LED_out(0);
     millis(50);
   }

   error = 0;

   loop_timer = get_cpu_usecs();                                                     //Set the timer for the next loop.
 }


void callibrate_level(void) {
  printf("level callibration\n");
  level_calibration_on = 1;

  while (channel_2 < 1100) {
    intr_handler();
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    millis(10);
  }
  LED_out(1);
  LED_out(0);

  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  for (error = 0; error < 64; error ++) {
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    gyro_signalen();
    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
    if (acc_y > 500 || acc_y < -500)error = 80;
    if (acc_x > 500 || acc_x < -500)error = 80;
    // micros(3700);
  }

  acc_pitch_cal_value /= 64;
  acc_roll_cal_value /= 64;

  LED_out(0);
  if (error < 80) {
    //EEPROM.write(0x10 + error, compass_cal_values[error]);
    for (error = 0; error < 15; error ++) {
      LED_out(1);
      millis(50);
      LED_out(0);
      millis(50);
    }
    error = 0;
  }
  else error = 3;
  level_calibration_on = 0;
  gyro_signalen();
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }
  angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
  loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
}


#endif //PATMOS_CALLIB_H