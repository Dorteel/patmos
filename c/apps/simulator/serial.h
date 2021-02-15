#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <machine/rtc.h>
#include <machine/patmos.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include "timer.h"

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)

unsigned char uart_data=0;
const int UART_LENGTH = 64;
const int UART_SEND_LENGTH = 60;

//Writes a byte to the UART2 (to be sent)
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

//Reads a byte from UART2 (from received data) and places it int the variable
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

void send_telemtry(char *xstr){
  char *START_STR = ".FPGA1";
  char *END_STR = "!";
  char full_message[UART_SEND_LENGTH];
  memset(full_message, '\0', UART_SEND_LENGTH);
  snprintf(full_message, UART_SEND_LENGTH, "%s%s%s",START_STR,xstr,END_STR);
  for(int i=0;i<UART_SEND_LENGTH;i++)
  {
    uart2_write(full_message[i]);
    millis(1);
  }
}

void receive_telemtry(char *Outstr){
  char START_IN[6] = "$PC";
  char temp[6]="";
  char full_message[UART_LENGTH]="";
  memset(full_message, '\0', UART_LENGTH);
  bool busy = true, start_temp = false, start = false;
  int k =0, comp = 0, cnt = 0, pos = 0 ;

    while((k<UART_LENGTH) && busy){ //(busy){
      uart2_read(&uart_data);
      // find end
      if((uart_data=='_')&&start){
        start = false;
        //printf("Message: %s\n",full_message);
        cnt=0;
        pos=0;
        busy = false;
      }
      // Appending message
      if(start){
        full_message[cnt]=uart_data;
        cnt++;
      }
      // find beginning
      if((uart_data=='$') && !start){
        pos = 0;
        start = true;
        cnt = 0;
      }
      millis(3);
      k++;
    }
    strcpy(Outstr, full_message);
}
