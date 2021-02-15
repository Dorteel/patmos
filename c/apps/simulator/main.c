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
//new

#include "serial.h"
#include "actuator_receiver.h"



int main(int argc, char **argv)
{
  printf("Hello Coppelia!\n");
  char *xstr = "";
  char sendUART[UART_SEND_LENGTH]="";
  char recUART[UART_LENGTH]="";
  for (int j=0;j<30;j++)
  {
    // Read receiver channels
    intr_handler();
    memset(sendUART, '\0', UART_SEND_LENGTH);
    sprintf(sendUART, "%d;%d;%d;%d;%d;%d;%d;", j, channel_1, channel_2, channel_3, channel_4, channel_5, channel_6);
    strcpy(xstr, sendUART);
    // send through uart
    printf("Sending nr.%d: %s. \n",j,xstr);
    send_telemtry(xstr);
    millis(200);
    receive_telemtry(recUART);
    printf("received: %s\n",recUART);
  }
  // Finish simulator
  sprintf(sendUART, "END_SIM");
  strcpy(xstr, sendUART);
  printf("Finish: %s. \n",xstr);
  send_telemtry(xstr);
  millis(100);
  return 0;
}
