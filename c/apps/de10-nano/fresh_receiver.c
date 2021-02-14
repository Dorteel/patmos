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

#define ITERATIONS 1000
__int32_t channel_1;
__int32_t channel_2;
__int32_t channel_3;
__int32_t channel_4;
__int32_t channel_5;
__int32_t channel_6;
__int32_t receiver_input[6];
int program_off=0;
int low[4]={1003,1004,1000,999}, center[4]={1493,1503,1496,1331}, high[4]={2002,2000,1993,1994};///(th,roll,pitch,yaw)

int cnt = 0;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


const unsigned int CPU_PERIOD = 20; //CPU period in ns.

//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

//Receiver controller register
#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )


//writes pwm signlas of width=data to the esc
void actuator_write(unsigned int actuator_id, unsigned int data)
{
    *(MOTOR + actuator_id) = data;
}

//get pulse width data from receiver
int receiver_read(unsigned int receiver_id){

    
    unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
    unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;
    return pulse_high_time;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
int convert_receiver_channel(unsigned int function)
{
  unsigned int  channel, reverse;       //First we declare some local variables
  int actual;                           ///(0,th,roll,pitch,yaw)
  int difference;

  if(function==0)
  {
    reverse = 0;                        //Reverse =1 when the transmitter channels are reversed, else 0
    channel =1;//roll
  }
  else if(function==1)
  {
    reverse = 1;
    channel=2;//pitch
  }
  else if(function==2)
  {
    reverse = 0;
    channel=0;//throttle
  }
  else
  {
    reverse = 0;                                            
    channel =3;//yaw
  }

  actual = receiver_input[channel];      //Read the actual receiver value for the corresponding function
  // low = 1000;  //Store the low value for the specific receiver input channel
  // center = 1500; //Store the center value for the specific receiver input channel
  // high = 2000;   //Store the high value for the specific receiver input channel

  if(actual < center[channel]){                                                         //The actual receiver value is lower than the center value
    if(actual < low[channel])actual = low[channel];                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center[channel]){                                                                        //The actual receiver value is higher than the center value
    if(actual > high[channel])actual = high[channel];                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}


// stores receiver values in an global array
void intr_handler_multi(){
    // read the receiver pwm duty cycle
  // __uint32_t timer = get_cpu_usecs();
  while(!program_off)
  {
      
      receiver_input[0] = receiver_read(0);
      receiver_input[1] = receiver_read(1);
      receiver_input[2] = receiver_read(2);
      receiver_input[3] = receiver_read(3);

      int channel_1_tmp = convert_receiver_channel(0);  //1(0)               //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
      int channel_2_tmp = convert_receiver_channel(1);  //2(1)               //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
      int channel_3_tmp = convert_receiver_channel(2);  //0(0)               //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
      int channel_4_tmp = convert_receiver_channel(3);  //3(0)               //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
      int channel_5_tmp = receiver_read(5);
      int channel_6_tmp = receiver_read(4);
      pthread_mutex_lock(&mutex);
      channel_1 = channel_1_tmp;
      channel_2 = channel_2_tmp;
      channel_3 = channel_3_tmp;
      channel_4 = channel_4_tmp;
      channel_5 = channel_5_tmp;
      channel_6 = channel_6_tmp;
      pthread_mutex_unlock(&mutex);
  }
}


void * work(void * arg) {
  for(int i = 0; i < ITERATIONS; i++) {
    pthread_mutex_lock(&mutex);
    cnt++;
    pthread_mutex_unlock(&mutex);
  }
  return NULL;
}


int main()
{
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

  int  loop_timer = get_cpu_usecs();
  while(!program_off)
  {
    pthread_mutex_lock(&mutex);
    pthread_mutex_unlock(&mutex);
    if(channel_3 < 1050 && channel_4 > 1950 && channel_1 < 1050 && channel_2 > 1950)
    {
      program_off = 1;
      
      printf("code stop\n");
    }
    
    printf("channel_1: %d\n",channel_1 );
    printf("time:%llu\n",get_cpu_usecs()-loop_timer);
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