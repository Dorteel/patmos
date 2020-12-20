//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_GYRO_H
#define PATMOS_GYRO_H

#include "../basic_lib/i2c_master.h"
#include "../basic_lib/timer.h"
#include "../basic_lib/led.h"

// Default I2C address for the MPU-6050 is 0x68.
#define MPU6050_I2C_ADDRESS 0x68

//MCU6050 registers
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_TEMP_OUT_H         0x41   // R
#define MPU6050_TEMP_OUT_L         0x42   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_GYRO_CONFIG        0x1B   // R
#define MPU6050_ACCEL_CONFIG       0x1C   // R
#define MPU6050_CONFIG_REG         0x1A   // R



void gyro_setup()
{
    //Setup the MPU-6050 registers
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00));                    //Set the register bits as 00000000 to activate the gyro
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08));                   //Set the register bits as 00001000 (500dps full scale)
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10));                  //Set the register bits as 00010000 (+/- 8g full scale range)
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)

}

void gyro_signalen()
{

    //Read the MPU-6050
    acc_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
    acc_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
    acc_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
    temperature = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
    gyro_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
    gyro_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
    gyro_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);

    if(cal_int == 2000)
    {
        gyro_axis[1] -= gyro_axis_cal[1];                                     //Only compensate after the calibration.
        gyro_axis[2] -= gyro_axis_cal[2];                                     //Only compensate after the calibration.
        gyro_axis[3] -= gyro_axis_cal[3];                                     //Only compensate after the calibration.
    }
    gyro_roll = gyro_axis[1];                                               //Set gyro_roll to the correct axis.
    gyro_pitch = gyro_axis[2];                                              //Set gyro_pitch to the correct axis.
    gyro_pitch *= -1;                                                       //Invert gyro_pitch to change the axis of sensor data.
    gyro_yaw = gyro_axis[3];                                                //Set gyro_yaw to the correct axis.
    gyro_yaw *= -1;                                                         //Invert gyro_yaw to change the axis of sensor data.


    acc_x = acc_axis[2];                                                    //Set acc_x to the correct axis.
    acc_x *= -1;                                                            //Invert acc_x.
    acc_y = acc_axis[1];                                                    //Set acc_y to the correct axis.
    acc_z = acc_axis[3];                                                    //Set acc_z to the correct axis.
    acc_z *= -1;                                                            //Invert acc_z.

}

void callibrate_gyro()
{
    cal_int = 0;                                                                        //Set the cal_int variable to zero.
    if (cal_int != 2000) {
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
            if (cal_int % 15 == 0) LED_out(1);                     //Change the led status every 125 readings to indicate calibration.
            gyro_signalen();                                                                //Read the gyro output.
            gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
            LED_out(0);                                                                       //Small delay to simulate a 250Hz loop during calibration.
        }
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
        gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
        gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
    }
    printf("gyro callibration done\n");

}


void vertical_acceleration_calculations(void) {
    acc_z_average_short_rotating_mem_location++;
    if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

    acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
    acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
    acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

    if (acc_z_average_short_rotating_mem_location == 0) {
        acc_z_average_long_rotating_mem_location++;

        if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

        acc_z_average_long_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
        acc_z_average_long[acc_z_average_long_rotating_mem_location] = acc_z_average_short_total / 25;
        acc_z_average_long_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
    }
    acc_z_average_total = acc_z_average_long_total / 50;


    acc_alt_integrated += acc_total_vector - acc_z_average_total;
    if (acc_total_vector - acc_z_average_total < 400 || acc_total_vector - acc_z_average_total > 400) {
        if (acc_z_average_short_total / 25 - acc_z_average_total < 500 && acc_z_average_short_total / 25 - acc_z_average_total > -500)
        {
            if(acc_alt_integrated > 200) acc_alt_integrated -= 200;
            else if(acc_alt_integrated < -200) acc_alt_integrated += 200;
        }
    }
}

#endif //PATMOS_GYRO_H
