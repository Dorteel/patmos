#ifndef IMU_H_   /* Include guard */
#define IMU_H_

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../../../i2c-master/i2c_master.h"

//
// ***********************************************
// ***************** M A C R O S *****************
//
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

//
// ***********************************************
// ************** V A R I A B L E S **************
//
unsigned int signature;

float dT;

// unsigned  int ACCEL_X_H;
// unsigned  int ACCEL_X_L;
// unsigned  int ACCEL_Y_H;
// unsigned  int ACCEL_Y_L;
// unsigned  int ACCEL_Z_H;
// unsigned  int ACCEL_Z_L;
// unsigned  int TEMP_L;
// unsigned  int TEMP_H;
// unsigned  int GYRO_X_H;
// unsigned  int GYRO_X_L;
// unsigned  int GYRO_Y_H;
// unsigned  int GYRO_Y_L;
// unsigned int GYRO_Z_H;
// unsigned  int GYRO_Z_L;

short int ACCEL_X_H;
short int ACCEL_X_L;
short int ACCEL_Y_H;
short int ACCEL_Y_L;
short int ACCEL_Z_H;
short int ACCEL_Z_L;
short int TEMP_L;
short int TEMP_H;
short int GYRO_X_H;
short int GYRO_X_L;
short int GYRO_Y_H;
short int GYRO_Y_L;
short int GYRO_Z_H;
short int GYRO_Z_L;

// Calibration variables
float gyro_x;
float gyro_y;
float gyro_z;

long int gyro_roll_cal;
long int gyro_pitch_cal;
long int gyro_yaw_cal;

float angle_pitch;
float angle_roll;
float angle_yaw;

float acc_total_vector;
float angle_pitch_acc;
float angle_roll_acc;

unsigned timer;

//
// ***********************************************
// ************** F U N C T I O N S **************
//
// void millis(int ms);
void gyro_setup(void);
void gyro_calibrate(void);
void gyro_compensated_read();
void gyro_read();

#endif // IMU_H_