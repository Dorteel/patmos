/*
    This program tests POSIX mutexes.

    Author: Torur Biskopsto Strom
    Copyright: DTU, BSD License
*/

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>
#include "Flight_controller_v2.h"
#include "telemetry/send_telemetry_data.h"
#include "gps/read_gps.h"
#include "basic_lib/actuator_receiver.h"

// Uncomment the line below to see the test fail
// when not using a mutex and cpucnt > 1
// #define WITHOUT_MUTEX 1
#define ITERATIONS 1000

#ifdef WITHOUT_MUTEX
_UNCACHED
#endif
int cnt = 0;

int inc_ch1(int ch1)
{
  // pthread_mutex_lock(&mutex);
  // int ch1=1;
  // pthread_mutex_unlock(&mutex);
  return ch1=ch1+1;
   
}
void * work(void * arg) {
  int id = get_cpuid();
  pthread_mutex_lock(&mutex);
  pthread_mutex_unlock(&mutex);
  int timer = get_cpu_usecs();
  while(!program_off){
    asm volatile ("" : : : "memory");
#ifndef WITHOUT_MUTEX
    pthread_mutex_lock(&mutex);
#endif
    cnt++;
#ifndef WITHOUT_MUTEX
    pthread_mutex_unlock(&mutex);
#endif
    while (get_cpu_usecs() - timer < 4000);
    timer = get_cpu_usecs();
  }
  return NULL;
}


void * work1(void * arg) {
  int id = get_cpuid();
  pthread_mutex_lock(&mutex);
  pthread_mutex_unlock(&mutex);
  int timer = get_cpu_usecs();
  while(!program_off){
    asm volatile ("" : : : "memory");
#ifndef WITHOUT_MUTEX
    pthread_mutex_lock(&mutex);
#endif
    channel_1 = inc_ch1(channel_1);
#ifndef WITHOUT_MUTEX
    pthread_mutex_unlock(&mutex);
#endif
    while (get_cpu_usecs() - timer < 4000);
    timer = get_cpu_usecs();
  }
  return NULL;
}


int main() {

  int cpucnt = 3;
  const int exp = cpucnt*ITERATIONS;
  
  printf("Started using %d threads\n",cpucnt);
  
  pthread_t *threads = malloc(sizeof(pthread_t) * cpucnt);
  
  // No thread starts before all are initialized;
  pthread_mutex_lock(&mutex);
  for(int i = 1; i < cpucnt;)
  {
    int retval = pthread_create(threads+i, NULL, work, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
     i++;
     retval = pthread_create(threads+i, NULL, intr_handler_multi, NULL);
    if(retval != 0)
    {
      printf("Unable to start thread %d, error code %d\n", i, retval);
      return retval;
    }
     i++;
  }
  pthread_mutex_unlock(&mutex);

///////////////////////main thread
  int timer = get_cpu_usecs();
  while(cnt < 990){
    printf("count: %d\n",cnt );
    printf("channel_!:%d\n",channel_1);
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
    while (get_cpu_usecs() - timer < 4000);
    timer = get_cpu_usecs();
    }
  program_off =1;
////////////////////////////

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
#ifndef WITHOUT_MUTEX
  // Locking to update the global state (get the newest value of cnt)
  pthread_mutex_lock(&mutex);
#endif
  printf("Expected count=%d, actual count=%d\n", exp, cnt);
#ifndef WITHOUT_MUTEX
  pthread_mutex_unlock(&mutex);
#endif
}
