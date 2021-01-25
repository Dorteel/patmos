// Created by rahul on 11/22/20.
//

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../basic_lib/i2c_master.h"

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

__uint8_t compass_address = 0x1E;            //The I2C address of the HMC5883L is 0x1E in hexadecimal form.
//LEDs register
#define LED ( *( ( volatile _IODEV unsigned * ) PATMOS_IO_LED ) )


float angle_roll_acc, angle_pitch_acc,angle_yaw, angle_pitch,angle_roll;
__uint8_t compass_calibration_on, heading_lock;
__int16_t compass_x, compass_y, compass_z;
__int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
__int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;
short int acc_axis[4], gyro_axis[4];
int gyro_axis_cal[4], acc_axis_cal[4];
__int16_t temperature;
__int16_t acc_x, acc_y, acc_z;
__int16_t gyro_pitch, gyro_roll, gyro_yaw;
int first_time=1, acc_count=0;
float pitch_offset=0,roll_offset=0;
int loop_timer;

__int32_t acc_total_vector;
float roll_level_adjust, pitch_level_adjust;
float declination = 0.0;                   //Set the declination between the magnetic and geographic north.
__int16_t cal_int;
float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
float dt = 0.02;
__uint8_t error;
__int16_t acc_pitch_cal_value;
__int16_t acc_roll_cal_value;
__uint8_t level_calibration_on;

void micros(int microseconds)
{
    unsigned int timer_ms = (get_cpu_usecs());
    unsigned int loop_timer = timer_ms;
    while(timer_ms - loop_timer < microseconds)timer_ms = get_cpu_usecs();
}

void millis(int milliseconds)
{
    unsigned int timer_ms = (get_cpu_usecs()/1000);
    unsigned int loop_timer = timer_ms;
    while(timer_ms - loop_timer < milliseconds)timer_ms = (get_cpu_usecs()/1000);
}

//Blinks the LEDs once
void blink_once()
{
    int i, j;
    for (i=2000; i!=0; --i)
        for (j=2000; j!=0; --j)
            LED = 0x0001;
    for (i=2000; i!=0; --i)
        for (j=2000; j!=0; --j)
            LED = 0x0000;
    return;
}

///1 to switch on LED and 0 to off
void LED_out(int i){
    if(i==1) LED = 0x0001;
    else LED = 0x0000;
    return;
}

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


void read_compass() {
    compass_y = i2c_reg8_read16b(compass_address,0x03);                 //Add the low and high byte to the compass_y variable.
    compass_y *= -1;                                              //Invert the direction of the axis.
    compass_z = i2c_reg8_read16b(compass_address,0x05);                 //Add the low and high byte to the compass_z variable.;
    compass_x = i2c_reg8_read16b(compass_address,0x07);                 //Add the low and high byte to the compass_x variable.;
    compass_x *= -1;                                              //Invert the direction of the axis.

//    printf("compasss: %d, %d, %d ", compass_x,compass_y,compass_z);
    //Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
    //variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
    if (compass_calibration_on == 0) {                            //When the compass is not beeing calibrated.
        compass_y += compass_offset_y;                              //Add the y-offset to the raw value.
        compass_y *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
        compass_z += compass_offset_z;                              //Add the z-offset to the raw value.
        compass_z *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
        compass_x += compass_offset_x;                              //Add the x-offset to the raw value.
    }

    //The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
    //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
    compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
    compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

    //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
    //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
    if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
    else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

    actual_compass_heading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0) actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360) actual_compass_heading -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
}

//At startup the registers of the compass need to be set. After that the calibration offset and scale values are calculated.
void setup_compass() {
    i2c_reg8_write8(compass_address,0x00,0x78);                                            //Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
    i2c_reg8_write8(compass_address,0x01,0x20);                                            //Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
    i2c_reg8_write8(compass_address,0x02,0x00);                                            //Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.

//Calculate the calibration offset and scale values
    compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
    compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

    compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
    compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
    compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}


//The following subrouting calculates the smallest difference between two heading values.
float course_deviation(float course_b, float course_c) {
    course_a = course_b - course_c;
    if (course_a < -180 || course_a > 180) {
        if (course_c > 180)base_course_mirrored = course_c - 180;
        else base_course_mirrored = course_c + 180;
        if (course_b > 180)actual_course_mirrored = course_b - 180;
        else actual_course_mirrored = course_b + 180;
        course_a = actual_course_mirrored - base_course_mirrored;
    }
    return course_a;
}

void callibrate_compass(void) {
    compass_calibration_on = 1;                                                //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
    LED_out(1);                                                             //The red led will indicate that the compass calibration is active.
    LED_out(0);                                                            //Turn off the green led as we don't need it.
                                             //Send telemetry data to the ground station.
    micros(3700);                                                 //Simulate a 250Hz program loop.
    read_compass();                                                          //Read the raw compass values.
    //In the following lines the maximum and minimum compass values are detected and stored.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
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
    level_calibration_on = 1;

    LED_out(1);
    LED_out(0);

    acc_pitch_cal_value = 0;
    acc_roll_cal_value = 0;

    for (error = 0; error < 64; error ++) {
        // send_telemetry_data();                                                   //Send telemetry data to the ground station.
        gyro_signalen();
        acc_pitch_cal_value += acc_y;
        acc_roll_cal_value += acc_x;
        if (acc_y > 500 || acc_y < -500)error = 80;
        if (acc_x > 500 || acc_x < -500)error = 80;
        micros(3700);
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

int main()
{
    callibrate_compass();
    callibrate_level();
    gyro_setup();
    printf("imu setup done");
    setup_compass();                                              //Initiallize the compass and set the correct registers.
    printf("compass setup done");
    read_compass();                                               //Read and calculate the compass data.
    angle_yaw = actual_compass_heading;                           //Set the initial compass heading.//
    printf("hello compass and imu");
    loop_timer = get_cpu_usecs();
    while(1)
    {
        gyro_signalen();
        read_compass();



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

        pitch_level_adjust = angle_pitch;                                           //Calculate the pitch angle correction.
        roll_level_adjust = angle_roll;                                             //Calculate the roll angle correction.

        printf("pitch:: %f  roll: %f yaw : %f actual_compass_heading: %f \n",angle_pitch,angle_roll,angle_yaw,actual_compass_heading);

        while (get_cpu_usecs() - loop_timer < 20000);                                            //We wait until 4000us are passed.
        loop_timer = get_cpu_usecs();



    }
    return 0;
}


