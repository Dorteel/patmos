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
  testing = FALSE;
  printf("\n ========================\n T E S T I N G\t B A R O M E T E R \n ------------------------\n");
  barometer_reset();
  printf("\n ======\n Reading coefficients: \n ------\n");
  barometer_setup();
  unsigned timediff = 0;
  printf("\n ======\n Reading values: \n ------\n");
  for(int i=0;i<100;i++){
    unsigned int start = (get_cpu_usecs());
    barometer_main();
    unsigned now = (get_cpu_usecs());
    printf("Runtime: %d\n",(now-start) );
    timediff += now-start;
}
    printf("%d Avg. runtime%d\n", timediff, (timediff/1000));
  return 0;
}
