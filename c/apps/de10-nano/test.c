#include "Flight_controller_v2.h"
#include "telemetry/send_telemetry_data.h"
#include "gps/read_gps.h"
#include "gyro/gyro.h"
#include "barometer/baro.h"
#include "compass/compass.h"
#include "basic_lib/actuator_receiver.h"
#include "pid/pid.h"
#include "safety/return_to_home.h"
#include "safety/led_signal.h"
#include "safety/start_stop_takeoff.h"
#include "callibration/callibration.h"
#include "basic_lib/analog_read.h"

const int scale = 16; // 2^16 = 65536
const int FractionMask = 0xffffffff >> (32 - scale);

#define DoubleToFixed(x) ((x) * (double)(1<<scale))
#define FixedToDouble(x) (((double)x) / (double)(1<<scale))
#define IntToFixed(x) (x << scale)
#define FixedToInt(x) (x >> scale)
#define FractionPart(x) (x & FractionMask)
// #define Multiply(x, y) (((long long)(x)*(long long)(y)) >> scale) // This is really slow, casting to 64 bits is not the best idea for speed
#define Multiply(x, y) ((((x)>>8)*((y)>>8))>>0) // This is faster, but end up losing precision

#define Divide(x, y) ((((x)>>8)*((y)>>8))>>0) // This is faster, but end up losing precision


void millis2(float ms)
{
  unsigned int timer_ms2 = (get_cpu_usecs()/1000);
  float loop_timer = (float)timer_ms2;
  while(timer_ms2 - loop_timer < ms)timer_ms2 = (get_cpu_usecs()/1000.0);
}

int main(int argc, char **argv)
{
  unsigned int start = (get_cpu_usecs());

  int f1 = DoubleToFixed(7.0);
  int f2 = DoubleToFixed(3.0);
  int y = Multiply(f1, f2);

  printf("answer: %f\n", FixedToDouble(y));
  

  unsigned now = (get_cpu_usecs());
  printf("\nRuntime: %d\n",(now-start) );
  return 0;
}