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
//  APP=pthread_mutex_test download

#define UART3 ((volatile _IODEV unsigned *)PATMOS_IO_UART3)

const unsigned int CPU_PERIOD = 20; //CPU period in ns.

unsigned char uart_data=0;

// This shitty fcn is from Arduino
void millis(int milliseconds)
{
  unsigned int timer_ms = (get_cpu_usecs()/1000);
  unsigned int loop_timer = timer_ms;
  while(timer_ms - loop_timer < milliseconds)timer_ms = get_cpu_usecs()/1000;
}


//Writes a byte to the uart2 (to be sent)
//Returns 0 is a character was sent, -1 otherwise.
int uart2_write(unsigned char data)
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
int uart2_read(unsigned char *data)
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



void send_telemtry(char *xstr){
  char *START_STR = ".FPGA1";
  char *END_STR = "!";
  char full_message[100];
  memset(full_message, '\0', 100);
  snprintf(full_message, 30, "%s%s%s",START_STR,xstr,END_STR);
  for(int i=0;i<30;i++)
  {
    uart2_write(full_message[i]);
    millis(1);
  }
}

void receive_telemtry(char *Outstr){
  char START_IN[6] = "$PC";
  char temp[6]="";
  char full_message[124]="";
  memset(full_message, '\0', 512);
  bool busy = true, start_temp = false, start = false;
  int k =0, comp = 0, cnt = 0, pos = 0 ;

    while((k<124) && busy){ //(busy){
      uart2_read(&uart_data);
      // find end
      if((uart_data=='-')&&start){
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

int main(int argc, char **argv)
{
  printf("Hello Telemetry!\n");
  char *xstr = "sample text 1";
  char recUART[512]="";
  for (int j=0;j<20;j++)
  {
    printf("\nWriting stuff nr.%d: %s\n",j,xstr);
    send_telemtry(xstr);
    millis(100);
    receive_telemtry(recUART);
    printf("received: %s\n",recUART);
  }

  return 0;
}
