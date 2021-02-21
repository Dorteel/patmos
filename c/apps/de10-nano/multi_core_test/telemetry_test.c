#include "../Flight_controller_v2.h"
#include "../gps/read_gps.h"
#include "../telemetry/send_telemetry_data.h"

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
