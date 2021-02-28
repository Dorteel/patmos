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
#include "../thread/i2c_thread_core1.h"
#include "../basic_lib/actuator_receiver.h"



int main() {

  int cpucnt = 3;
  
  printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);
  
  // No thread starts before all are initialized;
  pthread_mutex_lock(&mutex);
  for(int i = 1; i < cpucnt;)
  {
    int retval = pthread_create(threads+i, NULL, i2c_thread, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
    i++;
    retval = pthread_create(threads+i, NULL, intr_handler, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
    i++;
  }
  pthread_mutex_unlock(&mutex);
  loop_timer = get_cpu_usecs();
  while(!program_off)
  { 
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);

        //Stopping the code: throttle low and yaw right, roll left and pitch down
    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      ///need to add safety land and then switch off
      program_off = 1;
      break; 
    }

    printf("angle_pitch:%f angle_roll:%f angle_yaw:%f\n",angle_pitch, angle_roll, angle_yaw);
    printf("Pressure : %f \n", actual_pressure);
    // printf("angle_yaw : %f \n", angle_yaw);
    printf("actual_compass_heading:%f\n",actual_compass_heading );
    while (get_cpu_usecs() - loop_timer < dt*1000000);                                            //We wait until 4000us are passed.
    loop_timer = get_cpu_usecs();
  }
  
  for(int i = 1; i < cpucnt; i++) {
    void * dummy;
    int retval = pthread_join(*(threads+i), &dummy);
    if(retval != 0)
    {
      printf("Unable to join thread %d, error code %d\n", i, retval);
      return retval;
    }
  }
  
  free(threads);
  return 0;
}

