#include "baro.h"

/*
 * Function: millis
 * ----------------------------
 *  Waits for the given amount of ms
 *
 *  ms : amount of milliseconds to wait for
 *  
 *  Returns: None
 */
void millis(int ms)
{
  unsigned int timer_ms = (get_cpu_usecs()>>10);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < ms)timer_ms = (get_cpu_usecs()>>10);
}

/*
 * Function: barometer_reset
 * ----------------------------
 *   Sends a reset command
 *    "Shall be sent once after power-on"
 *
 *  Returns: None
 */
void barometer_reset(void)
{
  // Reset the barometer
  i2c_reg8_write8(MS5611_ADDR, 0x1E, (int)NULL);
  millis(5);
}

/*
 * Function: barometer_setup
 * ----------------------------
 *   Reads the calibration data from PROM
 *      C1 : Pressure sensitivity
 *      C2 : Pressure offset
 *      C3 : Temp coeff. for pressure sensitivity
 *      C4 : Temp. coeff. for pressure offset
 *      C5 : Reference temperature
 *      C6 : Temperature coeffifient for temperature
 *
 *  Returns: None
 */
void barometer_setup(void)
{
  // Read the coefficients (only done once)
  for (int i = 0; i < 6; i++) {
      Coff[i] = i2c_reg8_read16b(MS5611_ADDR, 0xA2 + i<<1);
      //printf("\tC[%i] = %lu\n",i, Coff[i]);
    }
}

/*
 * Function: barometer_adc
 * ----------------------------
 *   Starts the ADC conversion and reads the value
 *    the waiting time depends on the conversion resolution
 *   
 *   ADDR: the memory address (command) for the specific conversion
 *         Possible commands are descibed in the spec sheet of the MS5611
 *
 *  Returns: The converted (digital) pressure or temperature value
 */
unsigned long barometer_adc(char ADDR)
{
  i2c_reg8_write8(MS5611_ADDR, ADDR, (int)NULL);
  millis(10);
  unsigned long reading = i2c_reg8_read24b(MS5611_ADDR, 0x00);
  return reading;
}

/*
 * Function: barometer_main
 * ----------------------------
 *  Combines the above functions
 *    the waiting time depends on the conversion resolution
 *   
 *   ADDR: the memory address (command) for the specific conversion
 *         Possible commands are descibed in the spec sheet of the MS5611
 *
 *  Returns: None
 */
void barometer_main(void)
{
  unsigned long ptemp = barometer_adc(CMD_ADC_D1);
  unsigned long temp = barometer_adc(CMD_ADC_D2);
  //
  // ----------- Calculate temperature -----------
  //
  // Calculate difference between actual and reference temperature
  unsigned long dT = temp - ((Coff[4] << 8));
  // Actual temperature 
  // 20 degrees + (temp_difference * temperature sensitivity)
  temp = 2000 + (dT * (Coff[5] >> 23));
  //
  // ----------- Calculate temperature compensated pressure -----------
  //
  // Offset at actual temperature
  unsigned long long off = (Coff[1] << 16) + ((Coff[3] * dT) >> 7);
  // Sensitivity at actual temperature
  unsigned long long sens = (Coff[0] << 15) + ((Coff[2] * dT) >> 8);
  //
  // ----------- Second order temperature compensation -----------
  //
  if(temp < 2000)
  {
    Ti = (dT * dT) >> 31;
    offi = 5 * ((pow((temp - 2000), 2))) >> 1;
    sensi =  5 * ((pow((temp - 2000), 2))) >> 2; 
    
    if(temp < -1500)
    {
       offi = offi + 7 * ((pow((temp + 1500), 2)));      
       sensi = sensi + 11 * ((pow((temp + 1500), 2))) >> 2;
    }
  }
   
  temp -= Ti;
  off -= offi;
  sens -= sensi;
  //
  // ----------- Pressure calculation -----------
  //
  ptemp = ((ptemp * sens >> 21) - off);
  ptemp /= 32768;

  float pressure = ptemp / 100.0;
  float ctemp = temp / 100.0;
  float fTemp = ctemp * 1.8 + 32.0;

// ----------- Printing functions for comments -----------
//
  // printf("Temperature in Celsius : %f            ",ctemp);
  // printf(" Pressure : %f ",pressure);
  // printf(" mbar \n"); 
  // millis(10);
}
