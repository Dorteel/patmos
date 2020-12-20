#include "../Flight_conteroller_v2.h"
#include "../gps/read_gps.h"
#include "../gyro/gyro.h"
#include "../barometer/baro.h"
#include "../compass/compass.h"
#include "../basic_lib/actuator_receiver.h"
#include "../pid/pid.h"
#include "../safety/return_to_home.h"
#include "../safety/led_signal.h"
#include "../safety/start_stop_takeoff.h"
#include "../callibration/callibration.h"
#include "../basic_lib/analog_read.h"

#define transmitter false

int main(int argc, char **argv)
{
  printf("Hello MCU6050!\n");
  if(transmitter)
    intr_handler();

  unsigned int signature = 0;

  signature = i2c_reg8_read8(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I);
  printf("Signature = 0x%.2X\n", signature);
  gyro_setup();

  printf("gyro callibration start\n");
  callibrate_gyro();
  loop_timer = get_cpu_usecs();

  if(transmitter)
  {
    while(1){

      if(transmitter)
        intr_handler();
      if(transmitter)
      {
        if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
        {
          program_off = -1;
        }
      }
      
      gyro_signalen();
      printf("gyro data get\n");
      //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
      gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
      gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
      gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


      ////////////////////////////////////////////////////////////////////////////////////////////////////
      //This is the added IMU code from the videos:
      //https://youtu.be/4BoIE8YQwM8
      //https://youtu.be/j-kE0AMEWy4
      ////////////////////////////////////////////////////////////////////////////////////////////////////

      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += (gyro_pitch / 65.5)*dt;                                     //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += (gyro_roll / 65.5)*dt;                                       //Calculate the traveled roll angle and add this to the angle_roll variable.
      angle_yaw +=  (gyro_yaw / 65.5)*dt;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
      if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
      else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
      angle_pitch -= angle_roll * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the pitch angle to the roll angel.

       angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
      if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
      else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.


      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));        //Calculate the total accelerometer vector.


      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      if(abs(acc_y) < acc_total_vector){                                         //Prevent the asin function to produce a NaN
        angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;                  //Calculate the pitch angle.
      }
      if(abs(acc_x) < acc_total_vector){                                         //Prevent the asin function to produce a NaN
        angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;                  //Calculate the roll angle.
      }

      if(first_time)
      {
        pitch_offset = -angle_pitch_acc;                                         //start the pitch angle from 0 without any offsets
        roll_offset = -angle_roll_acc;                                           //start the roll angle from 0 without any offsets
      }

      angle_pitch_acc += pitch_offset;                                           //Accelerometer calibration value for pitch.
      angle_roll_acc += roll_offset;                                             //Accelerometer calibration value for roll.

      if( (int)angle_pitch_acc==0 && (int)angle_roll_acc==0)
      {
        acc_count++;
      }
      else acc_count=0;

      if(acc_count==20)first_time=0;    

      angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

      printf("pitch angle: %f  roll angle: %f \n",angle_pitch,angle_roll);

      if (get_cpu_usecs() - loop_timer > 20050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
      while (get_cpu_usecs() - loop_timer < 20000);                                            //We wait until 4000us are passed.
      loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
      
      if(transmitter)
        if(program_off==-1)break;                                                       //used to stop the code to reupload the program

    }
  }
  else
  {
    for (int i = 0; i < 1000; ++i)
    {

      if(transmitter)
        intr_handler();
      if(transmitter)
      {
        if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
        {
          program_off = -1;
        }
      }
      
      gyro_signalen();

      //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
      gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
      gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
      gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


      ////////////////////////////////////////////////////////////////////////////////////////////////////
      //This is the added IMU code from the videos:
      //https://youtu.be/4BoIE8YQwM8
      //https://youtu.be/j-kE0AMEWy4
      ////////////////////////////////////////////////////////////////////////////////////////////////////

      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += (gyro_pitch / 65.5)*dt;                                     //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += (gyro_roll / 65.5)*dt;                                       //Calculate the traveled roll angle and add this to the angle_roll variable.
      angle_yaw +=  (gyro_yaw / 65.5)*dt;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
      if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
      else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
      angle_pitch -= angle_roll * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the pitch angle to the roll angel.

       angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
      if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
      else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.


      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));        //Calculate the total accelerometer vector.


      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      if(abs(acc_y) < acc_total_vector){                                         //Prevent the asin function to produce a NaN
        angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;                  //Calculate the pitch angle.
      }
      if(abs(acc_x) < acc_total_vector){                                         //Prevent the asin function to produce a NaN
        angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;                  //Calculate the roll angle.
      }

      if(first_time)
      {
        pitch_offset = -angle_pitch_acc;                                         //start the pitch angle from 0 without any offsets
        roll_offset = -angle_roll_acc;                                           //start the roll angle from 0 without any offsets
      }

      angle_pitch_acc += pitch_offset;                                           //Accelerometer calibration value for pitch.
      angle_roll_acc += roll_offset;                                             //Accelerometer calibration value for roll.

      if( (int)angle_pitch_acc==0 && (int)angle_roll_acc==0)
      {
        acc_count++;
      }
      else acc_count=0;

      if(acc_count==20)first_time=0;    

      angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

      printf("pitch angle: %f  roll angle: %f \n",angle_pitch,angle_roll);

      if (get_cpu_usecs() - loop_timer > 20050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
      while (get_cpu_usecs() - loop_timer < 20000);                                            //We wait until 4000us are passed.
      loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
      
      if(transmitter)
        if(program_off==-1)break;                                                       //used to stop the code to reupload the program

    }
  }
  


  return 0;
}
