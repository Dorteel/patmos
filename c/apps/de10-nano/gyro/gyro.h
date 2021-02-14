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
    if(PRINT_COMMANDS)printf("gyro setup\n");
    //Setup the MPU-6050 registers
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00));                    //Set the register bits as 00000000 to activate the gyro
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08));                   //Set the register bits as 00001000 (500dps full scale)
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10));                  //Set the register bits as 00010000 (+/- 8g full scale range)
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)

}


void callibrate_gyro()
{
    if(PRINT_COMMANDS)printf("gyro callibration\n");
    cal_int = 0;                                                                        //Set the cal_int variable to zero.
    if (cal_int != 500) {
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        __uint32_t timer = get_cpu_usecs();
        for (cal_int = 0; cal_int < 500 ; cal_int ++) {                                  //Take 2000 readings for calibration.
            acc_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
            acc_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
            acc_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
            // temperature = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
            gyro_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
            gyro_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
            gyro_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);

            if(cal_int == 500)
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
            acc_z *= -1;                                                            //Invert acc_z.                                                                //Read the gyro output.
            gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
            // LED_out(0);                                                                       //Small delay to simulate a 250Hz loop during calibration.
            while (get_cpu_usecs() - timer < 0.02*1000000);
            timer = get_cpu_usecs();
        }
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_roll_cal /= 500;                                                            //Divide the roll total by 2000.
        gyro_pitch_cal /= 500;                                                           //Divide the pitch total by 2000.
        gyro_yaw_cal /= 500;                                                             //Divide the yaw total by 2000.
    }
    if(PRINT_COMMANDS)printf("gyro callibration done\n");

}

void gyro_read()
{
    //Read the MPU-6050
    acc_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
    acc_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
    acc_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
    temperature = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
    gyro_axis[1] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
    gyro_axis[2] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
    gyro_axis[3] = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
    if (level_calibration_on == 0) {
        acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
        acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
    }

    if(cal_int == 500)
    {
        gyro_axis[1] -= gyro_axis_cal[1];                                     //Only compensate after the calibration.
        gyro_axis[2] -= gyro_axis_cal[2];                                     //Only compensate after the calibration.
        gyro_axis[3] -= gyro_axis_cal[3];                                     //Only compensate after the calibration.
    }
}
void gyro_signalen()
{
    gyro_read();
    pthread_mutex_lock(&mutex);
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
    pthread_mutex_unlock(&mutex);

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += (gyro_pitch / 65.5)*dt;                                     //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += (gyro_roll / 65.5)*dt;                                       //Calculate the traveled roll angle and add this to the angle_roll variable.
    angle_yaw +=  (gyro_yaw / 65.5)*dt;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
    
    angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
    pthread_mutex_lock(&mutex);
    if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
    pthread_mutex_unlock(&mutex);
    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
    angle_pitch -= angle_roll * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_yaw * (dt/65.5)*(3.142/180));         //If the IMU has yawed transfer the pitch angle to the roll angel.

    pthread_mutex_lock(&mutex);
    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));        //Calculate the total accelerometer vector.


    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    
      angle_pitch_acc = asin(acc_y/acc_total_vector)* 57.296;                  //Calculate the pitch angle.
      angle_roll_acc = asin(acc_x/acc_total_vector)* -57.296;                  //Calculate the roll angle.
    pthread_mutex_unlock(&mutex);
    

    if(!first_angle){
      pthread_mutex_lock(&mutex);
    angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
    angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
    pthread_mutex_unlock(&mutex);
    first_angle = true;
    }
    else{
      pthread_mutex_lock(&mutex);
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      pthread_mutex_unlock(&mutex);
    }

    pthread_mutex_lock(&mutex);
    pitch_level_adjust = angle_pitch*15;                                           //Calculate the pitch angle correction.
    roll_level_adjust = angle_roll*15; 
    pthread_mutex_unlock(&mutex);

    vertical_acceleration_calculations();

    if(PRINT_COMMANDS)
    {
        pthread_mutex_lock(&mutex);
        printf("imu loop_timer: %llu\n",get_cpu_usecs() - timer );
        pthread_mutex_unlock(&mutex);
    }                                            

}


void vertical_acceleration_calculations(void) {
    acc_z_average_short_rotating_mem_location++;
    if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

    pthread_mutex_lock(&mutex);
    acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
    acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
    acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];
    pthread_mutex_unlock(&mutex);

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
