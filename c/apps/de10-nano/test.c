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

int main(int argc, char **argv)
{
  printf("Hello World!\n");
  return 0;
}