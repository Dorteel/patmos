#include "imu.h"

short FREQ = 25;
float SSF = 65.5; // Sensitivity Scale Factor, should be 65.5

unsigned int signature = 0;

short int ACCEL_X_H = 0;
short int ACCEL_Y_H = 0;
short int ACCEL_Z_H = 0;
short int TEMP_H = 0;
short int GYRO_X_H = 0;
short int GYRO_Y_H = 0;
short int GYRO_Z_H = 0;

short roll_compensation = 0;
short pitch_compensation = 0;
short yaw_compensation = 0;

float tau = 0.3;


void gyro_setup()
{
    //Setup the MPU-6050 registers
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00));                    //Set the register bits as 00000000 to activate the gyro
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x08));                   //Set the register bits as 00001000 (500dps full scale)
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10));                  //Set the register bits as 00010000 (+/- 8g full scale range)
    while(i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_CONFIG_REG, 0x03));                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
}


void gyro_read()
{
    ACCEL_Y_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
    ACCEL_X_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
    ACCEL_Z_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
    //TEMP_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
    GYRO_X_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
    GYRO_Y_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
    GYRO_Z_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);

    //printf("ACCEL_X_H: %hd,\t ACCEL_Y_H: %hd, ACCEL_Y_H: %hd\n", ACCEL_X_H, ACCEL_Y_H, ACCEL_Z_H);

}


void gyro_compensated_read()
{
  timer = get_cpu_usecs();
  gyro_read();
  GYRO_X_H -= gyro_roll_cal;
  GYRO_Y_H -= gyro_pitch_cal;
  GYRO_Z_H -= gyro_yaw_cal;

  // Convert to degrees / second
  // Also Divide by ferquency to get amount turned per frequency

  gyro_x = GYRO_X_H / (SSF*FREQ);
  gyro_y = GYRO_Y_H / (SSF*FREQ);
  gyro_z = GYRO_Z_H / (SSF*FREQ);


  angle_roll += gyro_x;
  angle_pitch += gyro_y;
  angle_yaw += gyro_z;


  //Angle coupling
  angle_roll += angle_pitch * sin(gyro_z*(1/(FREQ/SSF)) * (3.142/180));
  angle_pitch -= angle_roll * sin(gyro_z*((1/FREQ/SSF)) * (3.142/180));

  // if (angle_roll > 360) angle_roll -= 360;
  // if (angle_pitch > 360) angle_pitch -= 360;
  // if (angle_yaw > 360) angle_yaw -= 360;
  angle_roll = fmod(angle_roll, 360);
  angle_pitch = fmod(angle_pitch, 360);
  angle_yaw = fmod(angle_yaw, 360);

  // Calculate angle from accelerometer
  acc_total_vector = sqrt( (ACCEL_X_H*ACCEL_X_H) + (ACCEL_Y_H*ACCEL_Y_H) + (ACCEL_Z_H*ACCEL_Z_H)); // Total acceleration vector length
  angle_pitch_acc = asin( (float) ACCEL_Y_H / acc_total_vector) * 57.296; // Y component of the acceleration in degrees
  angle_roll_acc = asin( (float) ACCEL_X_H / acc_total_vector) * -57.296; // X component of the acceleration in degrees

  // Offset (needs calibrating)
  angle_roll_acc -= -0.5;
  angle_pitch_acc -= -1;

  // Complementary filter to correct for gyo drift

  angle_roll = angle_roll * (1-tau) + angle_roll_acc * tau;
  angle_pitch = angle_pitch * (1-tau) + angle_pitch_acc * tau;


  printf("Roll: %.5f\t Pitch: %.2f\t Yaw: %.2f \n", angle_roll, angle_pitch, angle_yaw);


  // Simulate loop time
  while((get_cpu_usecs()-timer) < ((1/FREQ)*1000000));
}


void gyro_calibrate()
{

  for (int cal_int = 0; cal_int < 4096 ; cal_int ++) {                                  //Take 2000 readings for calibration.
    
    gyro_read();                                                                //Read the gyro output.

    gyro_roll_cal += GYRO_X_H;                                                     //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += GYRO_Y_H;                                                   //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += GYRO_Z_H;                                                       //Ad yaw value to gyro_yaw_cal.
  }

  gyro_roll_cal = gyro_roll_cal>>12;                                                            //Divide the roll total by 2000.
  gyro_pitch_cal = gyro_pitch_cal>>12;                                                           //Divide the pitch total by 2000.
  gyro_yaw_cal = gyro_yaw_cal>>12;
  // printf("Calibration Values:\n\tRoll: %ld Pitch %ld: Yaw: %ld \n", gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal);

}
