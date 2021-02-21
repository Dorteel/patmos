#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../../i2c-master/i2c_master.h"
#include "imu/imu.h"

int main(int argc, char **argv)
{
  unsigned runtime = 4000; // 250Hz in microseconds
  printf("\n ========================\n T E S T I N G\t I M U \n ------------------------\n");
  printf("\n ======\n Setup \n ------\n");
  printf("... Setting up MPU-6050 \n");
  gyro_setup();
  printf("... Callibrating Gyro \n");
  gyro_calibrate();
  printf("\n ======\n Reading values \n ------\n");
  unsigned timediff = 0;
  

  for (int i = 0; i < 1000; i++)
  {
    // unsigned int start = (get_cpu_usecs());
    gyro_compensated_read();
    // printf("Roll: %.2f\t Pitch: %.2f\t Yaw: %.2f\n", angle_roll, angle_pitch, angle_yaw);
    // printf("X: %.d\t Y: %.d\t Z: %.d\n", ACCEL_X_H, ACCEL_Y_H, ACCEL_Z_H);
    // printf("Total vector: %f\t angle_pitch: %f\t angle roll: %f\n", acc_total_vector, angle_pitch_acc, angle_roll_acc);
    // unsigned now = (get_cpu_usecs());
    // printf("Loop time: %d\tus\n", now-start);
    //while(now - start < runtime) now = get_cpu_usecs();
    // timediff += now-start;
  }
  //printf("%d Avg. runtime%d\n", timediff, (timediff/1000));
  return 0;
}