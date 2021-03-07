#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <machine/patmos.h>
#include "../Flight_controller_v2.h"
#include "../gps/read_gps.h"
#include "../barometer/baro.h"
#include "../compass/compass.h"
#include "../gyro/gyro.h"
#include "../callibration/callibration.h"
#include "../basic_lib/actuator_receiver.h"
#include "../thread/gps_thread_core3.h"
#include "../thread/i2c_thread_core1.h"



int main() {

  int cpucnt = 4;
  int th_id, retval;
  printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);
  
  // No thread starts before all are initialized;
  pthread_mutex_lock(&mutex);
    th_id=1;
  
  retval = pthread_create(threads+th_id, NULL, i2c_thread, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }
  
  th_id++;
  retval = pthread_create(threads+th_id, NULL, gps_thread, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }
  th_id++;
  retval = pthread_create(threads+th_id, NULL, intr_handler, NULL);
  if(retval != 0)
  {
    printf("Unable to start thread %d, error code %d\n", th_id, retval);
    return retval;
  }

  pthread_mutex_unlock(&mutex);

  gps_setup();
  loop_timer = get_cpu_usecs();
  while(!program_off)
  { 

        //Stopping the code: throttle low and yaw right, roll left and pitch down
    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      ///need to add safety land and then switch off
      program_off = 1;
      printf("main loop end");
      break; 
    }
    // gps_get_lat_long();
    pthread_mutex_lock(&mutex);
    // printf("ch1: %d, ch2:%d, ch3:%d, ch4:%d\n", channel_1, channel_2, channel_3, channel_4);
    pthread_mutex_unlock(&mutex);
    // printf("angle_pitch:%f angle_roll:%f angle_yaw:%f\n",angle_pitch, angle_roll, angle_yaw);
    // printf("Pressure : %f \n", actual_pressure);
    // printf("angle_yaw : %f \n", angle_yaw);
    // printf("actual_compass_heading:%f\n",actual_compass_heading );
    while (get_cpu_usecs() - loop_timer < 40000);                                            //We wait until 4000us are passed.
    loop_timer = get_cpu_usecs();
  }
  
    void * dummy;
    th_id=2;
  
    printf("Waiting for threard nr.%d... ", th_id);
    retval = pthread_join(*(threads+th_id), &dummy);
    if(retval != 0)
    {
        printf("Unable to join thread %d, error code %d\n", th_id, retval);
    }  

    th_id=1;
    printf("Waiting for threard nr.%d... ", th_id);
    retval = pthread_join(*(threads+th_id), &dummy);
    if(retval != 0)
    {
        printf("Unable to join thread %d, error code %d\n", th_id, retval);
    }  
    th_id=3;
    printf("Waiting for threard nr.%d... ", th_id);
    retval = pthread_join(*(threads+th_id), &dummy);
    if(retval != 0)
    {
        printf("Unable to join thread %d, error code %d\n", th_id, retval);
    }  
  
  free(threads);
  return 0;
}

