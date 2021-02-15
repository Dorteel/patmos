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
// This is the value of the config value to set CH0-CH1 as differential, so the module:
// - VCC goes to 5V
// - CH1 and GND go to 0V
// - CH0 goes to the analog value
// const unsigned int ADC_CH0 = 0;
unsigned int ADC_CH0 = 0;   // 0x1000
//              S/D O/S S1 S0 UNI SLP
// bit position: 5   4   3  2  1   0
// unipolar CH0: 1   0   0  0  0   0   = 16
// polar CH0-1:  0   0   0  0  0   0   = 0


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
  printf("Analog reading test App.\n");

  printf("Config word --> read value\n");
ADC_CH0 = 1;

while(ADC_CH0<2147483648)
{

  for (int i=0; i<5; i++)
  {
    // First write the config word and send it to the ADC module
    write_adc(ADC_CH0);
  //  printf("config word = %d --> ", ADC_CH0);
    printf("    %d     -->     ", ADC_CH0);
   
    // Read what the module has measured acording to that configuration
    adc_val = read_adc();
    printf("%d\n",adc_val);
    millis(100);
  }

  ADC_CH0 = ADC_CH0*2;
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