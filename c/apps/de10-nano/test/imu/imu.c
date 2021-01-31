#include "imu.h"

unsigned int signature = 0;

unsigned int ACCEL_X_H = 0;
unsigned int ACCEL_X_L = 0;
unsigned int ACCEL_Y_H = 0;
unsigned int ACCEL_Y_L = 0;
unsigned int ACCEL_Z_H = 0;
unsigned int ACCEL_Z_L = 0;
unsigned int TEMP_L = 0;
unsigned int TEMP_H = 0;
unsigned int GYRO_X_H = 0;
unsigned int GYRO_X_L = 0;
unsigned int GYRO_Y_H = 0;
unsigned int GYRO_Y_L = 0;
unsigned int GYRO_Z_H = 0;
unsigned int GYRO_Z_L = 0;


void millis(int ms)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < ms)timer_ms = (get_cpu_usecs()/1000);
}


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
    ACCEL_X_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H);
    ACCEL_Y_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_YOUT_H);
    ACCEL_Z_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_ZOUT_H);
    //TEMP_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_TEMP_OUT_H);
    GYRO_X_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H);
    GYRO_Y_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_YOUT_H);
    GYRO_Z_H = i2c_reg8_read16b(MPU6050_I2C_ADDRESS, MPU6050_GYRO_ZOUT_H);

    gyro_x = GYRO_X_H;
    gyro_z = GYRO_Z_H;
    gyro_y = GYRO_Y_H;
}


void gyro_compensated_read()
{
  gyro_read();
  gyro_x -= gyro_roll_cal;
  gyro_y -= gyro_pitch_cal;
  gyro_z -= gyro_yaw_cal;

  // Convert to degrees / second
  //GYRO_X_H /= 65.5;
  //GYRO_Y_H /= 65.5;
  //GYRO_Z_H /= 65.5;
  
  //angle_roll += gyro_x * 0.0000611;;
  //angle_pitch += gyro_y * 0.0000611;;
  //angle_yaw += gyro_z * 0.0000611;;

  angle_roll += gyro_x >> 14;
  angle_pitch += gyro_y >> 14;
  angle_yaw += gyro_z >> 14;

  //printf("ACC_X = 0x%.4X (%d)\t", ACCEL_X_H, (short int)(ACCEL_X_H));
  //printf("ACC_Y = 0x%.4X (%d)\t", ACCEL_Y_H, (short int)(ACCEL_Y_H));
  //printf("ACC_Z = 0x%.4X (%d)\t", ACCEL_Z_H, (short int)(ACCEL_Z_H));
  //printf("TEMP = 0x%.4X (%.1f C)\t", TEMP_H,  ((double)((short int)(TEMP_H)) + 12412.0) / 340.0 ); //using datasheet formula for T in degrees Celsius
  // printf("GYRO_X  = %d\t\t", (GYRO_X_H));
  // printf("GYRO_Y  = %d\t\t", (GYRO_Y_H));
  // printf("GYRO_Z  = %d\t\n", (GYRO_Z_H));
  // printf("Roll  = %f\t\t", (angle_roll));
  // printf("Pitch  = %f\t\t",(angle_pitch));
  // printf("Yaw  = %f\n", (angle_yaw));

}


void gyro_calibrate()
{
  int max_z = 0;
  int max_i = 0;
  for (int cal_int = 0; cal_int < 2048 ; cal_int ++) {                                  //Take 2000 readings for calibration.
    
    gyro_read();                                                                //Read the gyro output.
    gyro_x = GYRO_X_H;
    gyro_z = GYRO_Z_H;
    gyro_y = GYRO_Y_H;
    // printf("%f,\t%d\t--\t", gyro_x, GYRO_X_H);
    // printf("%f,\t%d\t--\t", gyro_y, GYRO_Y_H);
    // printf("%f,\t%d\t--\t\n", gyro_z, GYRO_Z_H);
    gyro_roll_cal += gyro_x;                                                     //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_y;                                                   //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_z;                                                       //Ad yaw value to gyro_yaw_cal.
  }
  /*printf("un-compensated\t");
  printf("gyro_roll_cal  = %d\t\t", (short int)(gyro_roll_cal));
  printf("gyro_pitch_cal  = %d\t\t", (short int)(gyro_pitch_cal));
  printf("gyro_yaw_cal  = %d\t\n", (short int)(gyro_yaw_cal));
  */
  // degrees/s
  gyro_roll_cal = gyro_roll_cal>>11;                                                            //Divide the roll total by 2000.
  gyro_pitch_cal = gyro_pitch_cal>>11;                                                           //Divide the pitch total by 2000.
  gyro_yaw_cal = gyro_yaw_cal>>11;
  /*printf("compensated\t");                                                             //Divide the yaw total by 2000.
  printf("gyro_roll_cal  = %d\t\t", (short int)(gyro_roll_cal));
  printf("gyro_pitch_cal  = %d\t\t", (short int)(gyro_pitch_cal));
  printf("gyro_yaw_cal  = %d\t\n", (short int)(gyro_yaw_cal));
  */
}