//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_FLIGHT_CONTEROLLER_V2_H
#define PATMOS_FLIGHT_CONTEROLLER_V2_H

//standard header files
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>



float dt =0.04; /// loop timer in secs
#define battery_voltage_available 0// battery_voltage input from the fpga to compensate the esc input for change in battery volatge(will be later provided by DTU)
#define GYRO_CALLIB 1 //set to 1 to swtich on gyro callibration before flight
//channel 1- roll
// channel 2 - pitch
// channel 3- throttle
// channel 4- yaw

///motor configuration
// m4    ^     m1
//  \    |    /
//   \   |   /
//    \  |  /
//     \   /
//      \ /
//       /\
//      /  \
//     /    \
//    /      \
//   /        \
//m3/          \m2


#define STM32_board_LED PC13               //Change PC13 if the LED on the STM32 is connected to another output.

//Tuning parameters/settings is explained in this video: https://youtu.be/ys-YpOaA2ME
#define variable_1_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_2_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_3_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.

////////////////other variables
struct gps_tpv tpv;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const float pid_p_gain_roll = 0.43;               //Gain setting for the roll P-controller
const float pid_i_gain_roll = 0.0005;             //Gain setting for the roll I-controller
const float pid_d_gain_roll = 1.51;               //Gain setting for the roll D-controller
int pid_max_roll = 400;                           //Maximum output of the PID-controller (+/-)

const float pid_p_gain_pitch = pid_p_gain_roll;   //Gain setting for the pitch P-controller.
const float pid_i_gain_pitch = pid_i_gain_roll;   //Gain setting for the pitch I-controller.
const float pid_d_gain_pitch = pid_d_gain_roll;   //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;                          //Maximum output of the PID-controller (+/-)

const float pid_p_gain_yaw = 4;                   //Gain setting for the pitch P-controller. //4.0
const float pid_i_gain_yaw = 0.002;               //Gain setting for the pitch I-controller. //0.02
const float pid_d_gain_yaw = 0.0;                 //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                            //Maximum output of the PID-controller (+/-)


bool first_angle=false;

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;

float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).

float declination = 0.0;                   //Set the declination between the magnetic and geographic north.

__int16_t manual_takeoff_throttle = 0;    //Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).
__int16_t motor_idle_speed = 1100;           //Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI

__uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
__uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
__uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.

float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
int program_off=1;
__uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
__uint8_t check_byte, flip32, start;
__uint8_t error, error_counter, error_led;
__uint8_t flight_mode, flight_mode_counter, flight_mode_led;
__uint8_t takeoff_detected, manual_altitude_change;
__uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;
char recUART[512]="";
__uint8_t channel_select_counter;
__uint8_t level_calibration_on;
__uint32_t telemetry_buffer_byte;

__int16_t esc_1, esc_2, esc_3, esc_4;
__int16_t manual_throttle;
__int16_t throttle, takeoff_throttle, cal_int;
__int16_t temperature, count_var;
__int16_t acc_x, acc_y, acc_z;
__int16_t gyro_pitch, gyro_roll, gyro_yaw;

int low[4]={1003,1004,1000,999}, center[4]={1493,1503,1496,1331}, high[4]={2002,2000,1993,1994};///(th,roll,pitch,yaw)
__int32_t channel_1_start, channel_1, channel_1_base, pid_roll_setpoint_base;
__int32_t channel_2_start, channel_2, channel_2_base, pid_pitch_setpoint_base;
__int32_t channel_3_start, channel_3;
__int32_t channel_4_start, channel_4;
__int32_t channel_5_start, channel_5;
__int32_t channel_6_start, channel_6;
__int32_t measured_time, measured_time_start, receiver_watchdog,receiver_input[6];
__int32_t acc_total_vector, acc_total_vector_at_start;
__int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
__int16_t acc_pitch_cal_value;
__int16_t acc_roll_cal_value;

__int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
__int16_t acc_z_average_short[26], acc_z_average_long[51];

__uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

__int32_t acc_alt_integrated;

__uint32_t loop_timer, error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;

//Compass variables
__uint8_t compass_calibration_on, heading_lock;
__int16_t compass_x, compass_y, compass_z;
__int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
__int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;


//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
__uint16_t C[7];
__uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
__int64_t OFF, OFF_C2, SENS, SENS_C1, P;
__uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
__int32_t dT, dT_C5;
//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
__uint8_t parachute_rotating_mem_location;
__int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
__int32_t pressure_rotating_mem[50], pressure_total_avarage;
__uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

//GPS variables
__uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
__uint8_t waypoint_set, latitude_north, longiude_east ;
__uint16_t message_counter;
__int16_t gps_add_counter;
__int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
__int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
__uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
__uint8_t gps_rotating_mem_location, return_to_home_step;
__int32_t gps_lat_total_avarage, gps_lon_total_avarage;
__int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
__int32_t gps_lat_error, gps_lon_error;
__int32_t gps_lat_error_previous, gps_lon_error_previous;
__uint32_t gps_watchdog_timer;

float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
__uint8_t home_point_recorded;
__int32_t lat_gps_home, lon_gps_home;

short int acc_axis[4], gyro_axis[4];
int gyro_axis_cal[4], acc_axis_cal[4];

//Adjust settings online
__uint32_t setting_adjust_timer;
__uint16_t setting_click_counter;
__uint8_t previous_channel_6;
float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;


#endif //PATMOS_FLIGHT_CONTEROLLER_V2_H
