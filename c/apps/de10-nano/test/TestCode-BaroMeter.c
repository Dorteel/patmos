#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../../i2c-master/i2c_master.h"
#include "baro/baro.h"



int main(int argc, char **argv)
{
  printf("\n ========================\n T E S T I N G\t B A R O M E T E R \n ------------------------\n");
  barometer_reset();
  printf("\n ======\n Reading coefficients: \n ------\n");
  barometer_setup();
  int p;
  printf("\n ======\n Reading values: \n ------\n");
  for(int i=0;i<1000;i++){
    unsigned int start = (get_cpu_usecs());
    p = barometer_pressure();
    unsigned now = (get_cpu_usecs());
    printf("Runtime: %d\n",(now-start) );
    printf("Pressure: %d\n", p);
  }
  return 0;
}
