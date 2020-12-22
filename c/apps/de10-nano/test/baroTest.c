#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../../i2c-master/i2c_master.h"
#include "../baro/baro.h"


int main(int argc, char **argv)
{
  testing = TRUE;
  barometer_reset();
  barometer_setup();
  
  for(int i=0;i<10;i++)
    barometer_main();
  return 0;
}
