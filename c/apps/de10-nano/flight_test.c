#include "Flight_controller_v2.h"
#include "telemetry/send_telemetry_data.h"
#include "gps/read_gps.h"
#include "gyro/gyro.h"
#include "basic_lib/actuator_receiver.h"
#include "pid/pid.h"
#include "safety/return_to_home.h"
#include "safety/led_signal.h"
#include "safety/start_stop_takeoff.h"
#include "callibration/callibration.h"
#include "basic_lib/analog_read.h"
///PREDICT PROJECT FLIGHT CONTROLLER
//this code is for autonomous flight controller
// This code callibrates the IMU before start and stabilizes the drone using the RPY values from IMU. 
// It then takes inputs from the transmitter to have a 6DOF control of the drone 

//////////////////////////////////////////////////////////
// Procedure to start the drone:

// - go to t-crest/patmos folder
// - upload the code using the command "make APP=de10-IMU comp download"
// - wait for the program to upload
// - wait for the IMU to callibrate which is indicated by bliking LED on FPGA
// - unplug the upload cable
// - throttle low and yaw left to arm the motors
// - throttle low and yaw right to disarm the motors
// - stop the code before uploading, which is done by throttle low, yaw right and roll left and pitch back

///controls
 //                    throttle up                                             pitch forward
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |                          
 //                          |                                                       |       
 // yaw left -------------------------------yaw right       roll left -------------------------------roll right 
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                          |                                                       |
 //                    throttle down                                           pitch down


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

  intr_handler();                                                //Setup the timers for the receiver inputs and ESC's output.
  gps_setup();                                                  //Set the baud rate and output refreshrate of the GPS module.
  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.

  ////////////////// to be uncommented after composss node is made
  setup_compass();                                              //Initiallize the compass and set the correct registers.
  read_compass();                                               //Read and calculate the compass data.
  angle_yaw = actual_compass_heading;                           //Set the initial compass heading.
////////////////////
  //Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0) {                                 //Every 125 loops (500ms).
      blink_once();
    }
    millis(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.
  
  if(GYRO_CALLIB)callibrate_gyro();                                             //Calibrate the gyro offset.

  //Wait until the receiver is active.
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 4;                                                  //Set the error status to 4.
    LED_out(1);                                             //Show the error via the red LED.
    while (get_cpu_usecs() - loop_timer < dt*1000000);
    timer = get_cpu_usecs();                                                   //Delay 4ms to simulate a 250Hz loop
  }
  error = 0;                                                    //Reset the error status to 0.


  //When everything is done, turn off the led.
  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  if(battery_voltage_available==1)
  {
    battery_voltage = (float)batteryRead() / 112.81;
  }

  //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  barometer_setup();
////////////////////////////
  //Before starting the avarage accelerometer value is preloaded into the variables.
  for (start = 0; start <= 24; start++)acc_z_average_short[start] = acc_z;
  for (start = 0; start <= 49; start++)acc_z_average_long[start] = acc_z;
  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  start = 0;

  if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.

  loop_timer = get_cpu_usecs();                                        //Set the timer for the first loop.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  for (int j=0;j<10;j++)
  { 
    intr_handler();

        //Stopping the code: throttle low and yaw right, roll left and pitch down
    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      program_off = -1;
    }

    if(receiver_watchdog < 750)receiver_watchdog ++;
    if(receiver_watchdog == 750 && start == 2){
      channel_1 = 1500;
      channel_2 = 1500;
      channel_3 = 1500;
      channel_4 = 1500;
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
       if (channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900)callibrate_compass();
      //Level calibration move both sticks to the top left.
      if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)callibrate_level();
      //Change settings
      if (channel_6 >= 1900 && previous_channel_6 == 0) {
        previous_channel_6 = 1;
        if (setting_adjust_timer > get_cpu_usecs()/1000)setting_click_counter ++;
        else setting_click_counter = 0;
        setting_adjust_timer = get_cpu_usecs()/1000 + 1000;
        if (setting_click_counter > 3) {
          setting_click_counter = 0;
          change_settings();
        }
      }
      if (channel_6 < 1900)previous_channel_6 = 0;
    }

    heading_lock = 0;
    if (channel_6 > 1200)heading_lock = 1;                                           //If channel 6 is between 1200us and 1600us the flight mode is 2

    flight_mode = 1;                                                                 //In all other situations the flight mode is 1;
    if (channel_5 >= 1200 && channel_5 < 1600)flight_mode = 2;                       //If channel 6 is between 1200us and 1600us the flight mode is 2
    if (channel_5 >= 1600 && channel_5 < 1950)flight_mode = 3;                       //If channel 6 is between 1600us and 1900us the flight mode is 3
    if (channel_5 >= 1950 && channel_5 < 2100) {
      if (waypoint_set == 1 && home_point_recorded == 1 && start == 2)flight_mode = 4;
      else flight_mode = 3;
    }

    if (flight_mode <= 3) {
      return_to_home_step = 0;
      return_to_home_lat_factor = 0;
      return_to_home_lon_factor = 0;
    }

    return_to_home();                                                                //Jump to the return to home step program.
    flight_mode_signal();                                                            //Show the flight_mode via the green LED.
    LED_out(1);                                                                  //Show the error via the red LED.
    gyro_signalen();                                                                 //Read the gyro and accelerometer data.
    read_barometer();                                                                //Read and calculate the barometer data.
    read_compass();                                                                  //Read and calculate the compass data.
    if (gps_add_counter >= 0)gps_add_counter --;

    read_gps();

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

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
  angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;                  //Calculate the pitch angle.
  angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;                  //Calculate the roll angle.

  if(!first_angle){
  angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
  angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
  first_angle = true;
  }
  else{
  angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  }

    pitch_level_adjust = angle_pitch;                                           //Calculate the pitch angle correction.
    roll_level_adjust = angle_roll;                                             //Calculate the roll angle correction.

    vertical_acceleration_calculations();                                            //Calculate the vertical accelration.

    channel_1_base = channel_1;                                                      //Normally channel_1 is the pid_roll_setpoint input.
    channel_2_base = channel_2;                                                      //Normally channel_2 is the pid_pitch_setpoint input.
    gps_man_adjust_heading = angle_yaw;                                              //
    //When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
    //At startup the heading is registerd in the variable course_lock_heading.
    //First the course deviation is calculated between the current heading and the course_lock_heading.
    //Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
    if (heading_lock == 1) {
       heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
      channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
      channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
      gps_man_adjust_heading = course_lock_heading;

    }
    if (flight_mode >= 3 && waypoint_set == 1) {
      pid_roll_setpoint_base = 1500 + gps_roll_adjust;
      pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
    }
    else {
      pid_roll_setpoint_base = channel_1_base;
      pid_pitch_setpoint_base = channel_2_base;
    }

    //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
    if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
    if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
    if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
    if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;

    calculate_pid();                                                                 //Calculate the pid outputs based on the receiver inputs.

    start_stop_takeoff();                                                            //Starting, stopping and take-off detection

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //1410.1 = 112.81 / 0.08.
    if(battery_voltage_available)
    {
      battery_voltage = battery_voltage * 0.92 + batteryRead() / 1410.1;

    //Turn on the led if battery voltage is to low. Default setting is 10.5V
      if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)error = 1;

    }
    //The variable base_throttle is calculated in the following part. It forms the base throttle for every motor.
    if (takeoff_detected == 1 && start == 2) {                                         //If the quadcopter is started and flying.
      throttle = channel_3 + takeoff_throttle;                                         //The base throttle is the receiver throttle channel + the detected take-off throttle.
      if (flight_mode >= 2) {                                                          //If altitude mode is active.
        throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Creating the pulses for the ESC's is explained in this video:
    //https://youtu.be/Nju9rvZOjVQ
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    if (start == 2) {                                                                //The motors are started.
      if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
      esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
      esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
      esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
      esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

      if(battery_voltage_available==1)
      { 
        if (battery_voltage < 12.40 && battery_voltage > 6.0) {                        //Is the battery connected?
          esc_1 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-1 pulse for voltage drop.
          esc_2 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-2 pulse for voltage drop.
          esc_3 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-3 pulse for voltage drop.
          esc_4 += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-4 pulse for voltage drop.
        }
      }

      if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                        //Keep the motors running.
      if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;                        //Keep the motors running.
      if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;                        //Keep the motors running.
      if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;                        //Keep the motors running.

      if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
      if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
      if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
      if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
    }

    else {
      esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
      esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
      esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
      esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
    }


    actuator_write(m1, 1000);                                                 //give motors 1000us pulse.
    actuator_write(m2, 1000);
    actuator_write(m3, 1000);
    actuator_write(m4, 1000);

    send_telemetry_data();
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    //Because of the angle calculation the loop time is getting very important. If the loop time is
    //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
    //that the loop time is still 4000us and no longer! More information can be found on
    //the Q&A page:
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

    if (get_cpu_usecs() - loop_timer > 20050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
    while (get_cpu_usecs() - loop_timer < 20000);                                            //We wait until 4000us are passed.
    loop_timer = get_cpu_usecs();                                                           //Set the timer for the next loop.
    printf("loop_timer: %d\n", loop_timer );   //607326091 - 609668094 = 2342003
    printf("flight_mode: %d  start: %d\n",flight_mode, start );
  printf("esc1:%d esc2:%d esc3:%d esc4:%d\n",esc_1, esc_2, esc_3, esc_4 );
    if(program_off==-1)break;                                                       //used to stop the code to reupload the program

  }
  return 0;
}