#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "../basic_lib/uart.h"
#include "../basic_lib/timer.h"


void send_telemtry(char *xstr){
  char *START_STR = ".FPGA1";
  char *END_STR = "!";
  char full_message[100];
  memset(full_message, '\0', 100);
  snprintf(full_message, 30, "%s%s%s",START_STR,xstr,END_STR);
  for(int i=0;i<30;i++)
  {
    uart3_write(full_message[i]);
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
      uart3_read(&uart_data);
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void send_telemetry_data(void) {
  telemetry_loop_counter++;                                                                 //Increment the telemetry_loop_counter variable.
  if (telemetry_loop_counter == 1)telemetry_send_byte = 'J';                                //Send a J as start signature.
  if (telemetry_loop_counter == 2)telemetry_send_byte = 'B';                                //Send a B as start signature.
  if (telemetry_loop_counter == 3) {
    check_byte = 0;
    telemetry_send_byte = error;                              //Send the error as a byte.
  }
  if (telemetry_loop_counter == 4)telemetry_send_byte = flight_mode + return_to_home_step;                        //Send the flight mode as a byte.
  if (telemetry_loop_counter == 5)telemetry_send_byte = battery_voltage * 10;               //Send the battery voltage as a byte.
  if (telemetry_loop_counter == 6) {
    telemetry_buffer_byte = temperature;                                                    //Store the temperature as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the temperature variable.
  }
  if (telemetry_loop_counter == 7)telemetry_send_byte = telemetry_buffer_byte >> 8;         //Send the last 8 bytes of the temperature variable.
  if (telemetry_loop_counter == 8)telemetry_send_byte = angle_roll + 100;                   //Send the roll angle as a byte. Adding 100 prevents negative numbers.
  if (telemetry_loop_counter == 9)telemetry_send_byte = angle_pitch + 100;                  //Send the pitch angle as a byte. Adding 100 prevents negative numbers.
  if (telemetry_loop_counter == 10)telemetry_send_byte = start;                             //Send the error as a byte.
  if (telemetry_loop_counter == 11) {
    if (start == 2) {                                                                       //Only send the altitude when the quadcopter is flying.
      telemetry_buffer_byte = 1000 + ((float)(ground_pressure - actual_pressure) * 0.117);  //Calculate the altitude and add 1000 to prevent negative numbers.
    }
    else {
      telemetry_buffer_byte = 1000;                                                         //Send and altitude of 0 meters if the quadcopter isn't flying.
    }
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the altitude variable.
  }
  if (telemetry_loop_counter == 12)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the altitude variable.

  if (telemetry_loop_counter == 13) {
    telemetry_buffer_byte = 1500 + takeoff_throttle;                                        //Store the take-off throttle as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the take-off throttle variable.
  }
  if (telemetry_loop_counter == 14)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the take-off throttle variable.
  if (telemetry_loop_counter == 15) {
    telemetry_buffer_byte = angle_yaw;                                                      //Store the compass heading as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the compass heading variable.
  }
  if (telemetry_loop_counter == 16)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the last 8 bytes of the compass heading variable.
  if (telemetry_loop_counter == 17)telemetry_send_byte = heading_lock;                      //Send the heading_lock variable as a byte.
  if (telemetry_loop_counter == 18)telemetry_send_byte = number_used_sats;                  //Send the number_used_sats variable as a byte.
  if (telemetry_loop_counter == 19)telemetry_send_byte = fix_type;                          //Send the fix_type variable as a byte.
  if (telemetry_loop_counter == 20) {
    telemetry_buffer_byte = l_lat_gps;                                                      //Store the latitude position as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the latitude position variable.
  }
  if (telemetry_loop_counter == 21)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 22)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 23)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Send the next 8 bytes of the latitude position variable.
  if (telemetry_loop_counter == 24) {
    telemetry_buffer_byte = l_lon_gps;                                                      //Store the longitude position as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the longitude position variable.
  }
  if (telemetry_loop_counter == 25)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the longitude position variable.
  if (telemetry_loop_counter == 26)telemetry_send_byte = telemetry_buffer_byte >> 16;       //Send the next 8 bytes of the longitude position variable.
  if (telemetry_loop_counter == 27)telemetry_send_byte = telemetry_buffer_byte >> 24;       //Send the next 8 bytes of the longitude position variable.

  if (telemetry_loop_counter == 28) {
    telemetry_buffer_byte = adjustable_setting_1 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 1.
  }
  if (telemetry_loop_counter == 29)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 1.
  if (telemetry_loop_counter == 30) {
    telemetry_buffer_byte = adjustable_setting_2 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 2.
  }
  if (telemetry_loop_counter == 31)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 2.
  if (telemetry_loop_counter == 32) {
    telemetry_buffer_byte = adjustable_setting_3 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    telemetry_send_byte = telemetry_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 3.
  }
  if (telemetry_loop_counter == 33)telemetry_send_byte = telemetry_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 3.


  if (telemetry_loop_counter == 34)telemetry_send_byte = check_byte;                        //Send the check-byte.


  //After 125 loops the telemetry_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (telemetry_loop_counter == 125)telemetry_loop_counter = 0;                             //After 125 loops reset the telemetry_loop_counter variable

  if(telemetry_loop_counter <=34)
  {

    char text[512]; /* create a place to store text */
    sprintf(text,"%d", telemetry_send_byte); /* create textual representation of 'telemetry_send_byte' */
    /* and store it in the array 'text' */
    char *xstr = text;
    send_telemtry(xstr);
  }
}