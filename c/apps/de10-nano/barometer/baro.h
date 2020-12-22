//
// Created by rahul on 11/22/20.
//

#ifndef PATMOS_BARO_H
#define PATMOS_BARO_H

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../basic_lib/i2c_master.h"
#include "../basic_lib/timer.h"

//Barometer v2 variables
#define BARO_REG        0xA0     // Register address for coeffifient
#define MS5611_ADDR     0x77
#define CMD_ADC_D1      0x48     // Pressure - 4096 resolution
#define CMD_ADC_D2      0x58     // Temperature - 4096 resolution

void read_barometer(void)
{
    barometer_counter ++;

    if (barometer_counter == 1) {
        if (temperature_counter == 0) {
            //Get temperature data from MS-5611
            raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
            raw_temperature_rotating_memory[average_temperature_mem_location] = i2c_reg8_read24b(MS5611_ADDR, 0x00);
            raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
            average_temperature_mem_location++;
            if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
            raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
        }
        else {
            //Get pressure data from MS-5611
            raw_pressure = i2c_reg8_read24b(MS5611_ADDR, 0x00);
        }

        temperature_counter ++;
        if (temperature_counter > 9) {
            temperature_counter = 0;
            //Request temperature data
            i2c_reg8_write8_empty(MS5611_ADDR, 0x58);
        }
        else {
            //Request pressure data
            i2c_reg8_write8_empty(MS5611_ADDR, 0x48);
        }
    }
    if (barometer_counter == 2) {
        //Calculate pressure as explained in the datasheet of the MS-5611.
        dT = C[5];
        dT <<= 8;
        dT *= -1;
        dT += raw_temperature;

        OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);

        SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);

        P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

        if (actual_pressure == 0) {
            actual_pressure = P;
            actual_pressure_fast = P;
            actual_pressure_slow = P;
        }

        actual_pressure_fast = actual_pressure_fast * (float)0.92 + P * (float)0.08;
        actual_pressure_slow = actual_pressure_slow * (float)0.99 + P * (float)0.01;
        actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
        if (actual_pressure_diff > 8)actual_pressure_diff = 8;
        if (actual_pressure_diff < -8)actual_pressure_diff = -8;
        if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
        actual_pressure = actual_pressure_slow;
    }
    if (barometer_counter == 2) {                                                                               //When the barometer counter is 3

        barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
        //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
        //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
        if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
        parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
        parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
        parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
        pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
        parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
        if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

        if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
            if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
            //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
            //will indicate if the altitude of the quadcopter is changed by the pilot.
            manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
            manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
            if (channel_3 > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
                manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
                pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
                manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
            }
            if (channel_3 < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
                manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
                pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
                manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
            }

            //Calculate the PID output of the altitude hold.
            pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
            pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

            //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
            //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
            pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
            if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
                pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
                if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
            }

            //In the following section the I-output is calculated. It's an accumulation of errors over time.
            //The time factor is removed as the program loop runs at 250Hz.
            pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
            if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
            else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
            //In the following line the PID-output is calculated.
            //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
            //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
            //D = pid_d_gain_altitude * parachute_throttle.
            pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
            //To prevent extreme PID-output the output must be limited.
            if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
            else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
        }

            //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
        else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
            pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
            pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
            pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
            manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
            manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
        }
    }
}


void barometer_setup()
{
    for (start = 1; start <= 6; start++) {
        C[start] = i2c_reg8_read16b(MS5611_ADDR,0xA0 + start * 2);                //Add the low and high byte to the C[x] calibration variable.
    }

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

    //The MS5611 needs a few readings to stabilize.
    for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
        read_barometer();                                           //Read and calculate the barometer data.
        millis(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
    }
    actual_pressure = 0;                                          //Reset the pressure calculations.

#endif //PATMOS_BARO_H
