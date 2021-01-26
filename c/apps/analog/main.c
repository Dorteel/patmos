#include <stdio.h>
#include <machine/patmos.h>

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <math.h>
#include <time.h>

//SPI
#define ADC ((volatile _IODEV unsigned *)0xf00e0000)
// 0xf00e0000 = PATMOS_IO_ETH1 in patmos.h

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

// This shitty fcn is from Arduino
void millis(int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = get_cpu_usecs()/1000;
}


unsigned int reverseBits(unsigned int n)
{
  unsigned int rev = 0;
  while(n > 0)
  {
    rev <<= 1;
    if((n&1) == 1)
    {
      rev ^= 1;
    }
    n >>= 1;
  }

  return rev;
}

void write_adc(unsigned int config_word)
{
  *(ADC) = config_word;
}



int read_adc()
{
  return reverseBits(*(ADC));
}


int main(int argc, char **argv)
{
  unsigned int adc_val;
  unsigned int adc_val_wr = 0;
  printf("Analog reading test App.\n");

  for (int i=0; i<10; i++)
  {
    //adc_val_wr = (unsigned int)i;
    write_adc(adc_val_wr);
    printf("Analog writing: %d\n", adc_val_wr);
    adc_val = read_adc();
    printf("Analog read value: %d\n",adc_val);
    millis(500);
  }

  return 0;
}



// Orignal code
// void write_adc()
// {
//   unsigned int config_word = 0;
//   *(ADC) = config_word;
// }

// int main(int argc, char **argv)
// {
//   unsigned int adc_val;
//   printf("Analog reading test App.\n");

//   while (1)
//   {
//     write_adc();
//     printf("Writing...\n");
//     adc_val = read_adc();
//     printf("Analog read value: %d\n",adc_val);
//   }

//   return 0;
// }