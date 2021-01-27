//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_UART_H
#define PATMOS_UART_H
#include <stdio.h>
#include <stdlib.h>
#include <machine/patmos.h>
#include <machine/exceptions.h>
#include <stdbool.h>
#include <math.h>
#include <machine/rtc.h>

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)
#define UART3 ((volatile _IODEV unsigned *)PATMOS_IO_UART3)


unsigned char uart_data=0;


//Writes a byte to the uart2 (to be sent)
//Returns 0 is a character was sent, -1 otherwise.
int uart2_write(unsigned char data)
{
    if ((*UART2 & 0x00000001) != 0)
    {
        *UART2 = (unsigned int)data;
        return 1;
    }
    else
    {
        data = 0;
        return 0;
    }
}

//Reads a byte from uart2 (from received data) and places it int the variable
//specified by the pointer * data.
//Returns 0 is a character was read, -1 otherwise.
int uart2_read(unsigned char *data)
{
    if ((*UART2 & 0x00000002) != 0)
    {
        *data = (unsigned char)(*(UART2 + 1) & 0x000000FF);
        return 1;
    }
    else
    {
        *data = 0;
        return 0;
    }
}


//Writes a byte to the uart2 (to be sent)
//Returns 0 is a character was sent, -1 otherwise.
int uart3_write(unsigned char data)
{
  if ((*UART3 & 0x00000001) != 0)
  {
    *UART3 = (unsigned int)data;
    return 1;
  }
  else
  {
    data = 0;
    return 0;
  }
}

//Reads a byte from uart2 (from received data) and places it int the variable
//specified by the pointer * data.
//Returns 0 is a character was read, -1 otherwise.
int uart3_read(unsigned char *data)
{
  if ((*UART3 & 0x00000002) != 0)
  {
    *data = (unsigned char)(*(UART3 + 1) & 0x000000FF);
    return 1;
  }
  else
  {
    *data = 0;
    return 0;
  }
}



#endif //PATMOS_UART_H
