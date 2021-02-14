#ifndef BARO_H_   /* Include guard */
#define BARO_H_

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include "../../../i2c-master/i2c_master.h"

//
// ***********************************************
// ***************** M A C R O S *****************
//
#define TRUE            1        
#define FALSE           0
#define BARO_REG        0xA0     // Register address for coeffifient
#define MS5611_ADDR     0x77
#define CMD_ADC_D1      0x48     // Pressure - 4096 resolution
#define CMD_ADC_D2      0x58     // Temperature - 4096 resolution

//
// ***********************************************
// ************** V A R I A B L E S **************
//
unsigned long offi;
unsigned long sensi;
unsigned long Ti;
unsigned long BaroMeterCoff[6];
unsigned int data[3];
//
// ***********************************************
// ************** F U N C T I O N S **************
//
void millis(int ms);
void barometer_reset(void);
void barometer_setup(void);
unsigned long barometer_adc(char ADDR);
int barometer_pressure(void);
#endif // BARO_H_