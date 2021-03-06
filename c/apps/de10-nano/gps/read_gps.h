//
// Created by rahul on 11/8/20.
//


#ifndef PATMOS_READ_GPS_H
#define PATMOS_READ_GPS_H


#include "gps.h"
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include "../basic_lib/timer.h"
#include "../basic_lib/uart.h"
#include <time.h>

int loop_counter;
unsigned char gps_data=0;

struct gps_tpv tpv;
int result;
//---- to filter Data
char str[500];
int str_i[500];
char str_c[500];
char cRMC[6] = "$GNRMC";
char cVTG[6] = "$GNVTG";
char cGGA[6] = "$GNGGA";
char cGGL[6] = "$GNGGL";
char cGSA[6] = "$GNGSA";
char cGPSm[10] = "";    // for the GPS Mode 
// Drone GPS = GNRMC, NEO6M GPS = GPRMC


#define error_print false


void gps_setup(void)
{
    if(PRINT_COMMANDS)printf("gps setup\n");
    gps_init_tpv(&tpv);
}


void read_gps(void) {
    int loop_counter = 0;
    //------ to filter data ------------------
    int str_temp[6], str_mode = 0, tmpMode = 0;
    char str_tempc[6], str_tempMode[10];
    int start_temp = 0, start_mode = 0;
    int end_temp = 6, end_mode = 10;
    bool b_temp = false;
    bool printed = false;
    bool equal_RMC = false;
    int start_c = 6;
    int end_c = 300;
    bool equal_VTG = false, equal_GSA = false;


    // millis(250);

    for(loop_counter=0;loop_counter<1000;loop_counter++) {     
        millis(4);                                                    //Stay in this loop until the data variable data holds a q.
            // for(int j=0;j<300;j++){
                // pthread_mutex_lock(&mutex);
                // printf("%c",gps_data);
                // pthread_mutex_unlock(&mutex);
                // uart2_read(&gps_data);
                // if(program_off) break;
                // if (gps_data == 36) {
                //     b_temp = true;
                // }
                // if (b_temp && (start_temp < end_temp)) {
                //     str_temp[start_temp] = gps_data;
                //     str_tempc[start_temp] = (char) gps_data;
                //     start_temp++;
                // }

                // if (equal_RMC && (start_c < end_c)) {
                //     if ((char) gps_data != '\0'){
                //     str_c[start_c] = (char) gps_data;
                //     start_c++;
                //     }
                // }
                // //find the RMC string
                // if ((start_temp == end_temp) && !equal_RMC) {
                //     b_temp = false;
                //     int comp = 0;
                //     for (int j = 0; j < 6; j++) {
                //         comp = comp + str_tempc[j] - cRMC[j];
                //         str_c[j] = str_tempc[j];
                //     }
                //     if (comp == 0) {
                //         equal_RMC = true;
                //     }
                //     start_temp = 0; // Try again
                // }

                // //find the VTG string
                // if ((start_temp == end_temp) && equal_RMC && !equal_VTG) {
                //     b_temp = false;
                //     int comp = 0;
                //     for (int j = 0; j < 6; j++) {
                //         comp = comp + str_tempc[j] - cVTG[j];
                //     }
                //     if (comp == 0) {
                //         equal_VTG = true;
                //           //printf("\n\n");
                //     } else {
                //         equal_RMC = false; //If the next string is not VTG, it must go back false
                //         start_c = 6;
                //     }
                //     start_temp = 0; // Try again?
                // }

                // //find the GSA string
                // if ((start_temp == end_temp) && equal_RMC && !equal_GSA) {
                //     b_temp = false;
                //     int comp = 0;
                //     for (int j = 0; j < 6; j++) {
                //         comp = comp + str_tempc[j] - cGSA[j];
                //         str_tempMode[j] = str_tempc[j];
                //         str_mode++;
                //     }
                //     if (comp == 0) {
                //         equal_GSA = true;
                //         //printf("CGSA header found");
                //     }else{
                //         str_mode = 0;
                //     }
                //     start_temp = 0; // Try again
                // } 
                // else if (equal_GSA && (str_mode<10))
                // {
                //      str_tempMode[str_mode] = (char) gps_data;
                //      str_mode++;
                // }else if (equal_GSA && (str_mode == 10) && (tmpMode == 0))
                // {
                //     tmpMode = (int)str_tempMode[9] - '0';
                // }
            // }
            pthread_mutex_lock(&mutex);
            pthread_mutex_unlock(&mutex);
            if(program_off){
                break;
            }
    }
    micros(10);
    pthread_mutex_lock(&mutex);
    printf("programm off_gps:%d\n", program_off);
    // printf("%s\n", str_c);
    pthread_mutex_unlock(&mutex);
    //printf("\n!!!!END!!!\n");
    // printf("Mode = %s, %c, %d\n", str_tempMode, str_tempMode[9], tmpMode);


    // result = gps_decode(&tpv, str_c);
    // if (result != GPS_OK && error_print)
    // {
    //     printf(stderr, "Error (%d): %s\n", result, gps_error_string(result));
    //     //  return EXIT_FAILURE;
    // }
    // // Overwrite value of mode with the tmp one
    // if(equal_GSA && (str_mode == 10)){
    //     tpv.mode = tmpMode;
    // }

    // // pthread_mutex_unlock(&mutex);
    // //pthread_mutex_lock(&mutex);
    // lat_gps_actual = abs((double)tpv.latitude/GPS_LAT_LON_FACTOR);
    // lon_gps_actual = abs((double)tpv.longitude/GPS_LAT_LON_FACTOR);

    // if(PRINT_COMMANDS)printf("GPS = %d, %d, mode = %d\n", lat_gps_actual, lon_gps_actual, tpv.mode);
    //pthread_mutex_unlock(&mutex);
    
    // double alt =  (double)tpv.altitude/GPS_VALUE_FACTOR;

    // if (tpv.latitude >0)latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
    // else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

    // if (tpv.longitude >0)longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
    // else longiude_east = 0;

    // if(tpv.mode == 3)
    // {
    //     pthread_mutex_lock(&mutex);
    //     number_used_sats =8;                                             //Filter the number of satillites from the GGA line.
    //     pthread_mutex_unlock(&mutex);
    // }



    // if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
    //     lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
    //     lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
    // }

    // lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Divide the difference between the new and previous latitude by ten.
    // lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Divide the difference between the new and previous longitude by ten.

    // l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
    // l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

    // lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
    // lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.

    // //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
    // gps_add_counter = 5;                                                                               //Set the gps_add_counter variable to 5 as a count down loop timer
    // new_gps_data_counter = 9;                                                                          //Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
    // lat_gps_add = 0;                                                                                   //Reset the lat_gps_add variable.
    // lon_gps_add = 0;                                                                                   //Reset the lon_gps_add variable.
    // new_gps_data_available = 1;                                                                        //Set the new_gps_data_available to indicate that there is new data available.

    // // printf("lat_gps_actual: %f  lon_gps_actual: %f\n",(double)tpv.latitude,(double)tpv.longitude );
    // if (new_gps_data_available) {                                                                           //If there is a new set of GPS data available.
    //     if (number_used_sats < 8){if(PRINT_COMMANDS && error_print)printf("not enough satellite to lock on");}                                                              //Turn the LED on the STM solid on (LED function is inverted). Check the STM32 schematic.
    //     gps_watchdog_timer = get_cpu_usecs();                                                                        //Reset the GPS watch dog tmer.
    //     new_gps_data_available = 0;                                                                           //Reset the new_gps_data_available variable.

    //     if (flight_mode >= 3 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
    //         pthread_mutex_lock(&mutex);
    //         waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
    //         l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
    //         l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
    //         pthread_mutex_unlock(&mutex);
    //     }

    //     if (flight_mode >= 3 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
    //         //GPS stick move adjustments
    //         if (flight_mode == 3 && takeoff_detected == 1) {
    //             if (!latitude_north) {
    //                 l_lat_gps_float_adjust += 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //South correction
    //             }
    //             else {
    //                 l_lat_gps_float_adjust -= 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //North correction
    //             }

    //             if (!longiude_east) {
    //                 l_lon_gps_float_adjust -= (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //West correction
    //             }

    //             else {
    //                 l_lon_gps_float_adjust += (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //East correction
    //             }
    //         }

    //         if (l_lat_gps_float_adjust > 1) {
    //             l_lat_waypoint ++;
    //             l_lat_gps_float_adjust --;
    //         }
    //         if (l_lat_gps_float_adjust < -1) {
    //             l_lat_waypoint --;
    //             l_lat_gps_float_adjust ++;
    //         }

    //         if (l_lon_gps_float_adjust > 1) {
    //             l_lon_waypoint ++;
    //             l_lon_gps_float_adjust --;
    //         }
    //         if (l_lon_gps_float_adjust < -1) {
    //             l_lon_waypoint --;
    //             l_lon_gps_float_adjust ++;
    //         }

    //         gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Calculate the latitude error between waypoint and actual position.
    //         gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Calculate the longitude error between waypoint and actual position.

    //         gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
    //         gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
    //         gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.

    //         gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
    //         gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
    //         gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
    //         gps_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    //         if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.

    //         gps_lat_error_previous = gps_lat_error;                                                             //Remember the error for the next loop.
    //         gps_lon_error_previous = gps_lon_error;                                                             //Remember the error for the next loop.

    //         //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
    //         //The Proportional part = (float)gps_lat_error * gps_p_gain.
    //         //The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
    //         gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
    //         gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

    //         if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
    //         if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

    //         //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
    //         pthread_mutex_lock(&mutex);
    //         gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
    //         gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

    //         //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
    //         if (gps_roll_adjust > 300) gps_roll_adjust = 300;
    //         if (gps_roll_adjust < -300) gps_roll_adjust = -300;
    //         if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
    //         if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    //         pthread_mutex_unlock(&mutex);
    //     }
    // }

    // if (gps_watchdog_timer + 1000 < get_cpu_usecs()) {                                                             //If the watchdog timer is exceeded the GPS signal is missing.
    //     if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
    //         pthread_mutex_lock(&mutex);
    //         flight_mode = 2;                                                                                    //Set the flight mode to 2.
    //         pthread_mutex_unlock(&mutex);
    //         error = 4;                                                                                          //Output an error.
    //     }
    // }

    // if (flight_mode < 3 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
    //     pthread_mutex_lock(&mutex);
    //     gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
    //     gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
    //     if (waypoint_set == 1) {                                                                              //If the waypoints are stored
    //         gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the
    //         waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    //     }
    //     pthread_mutex_unlock(&mutex);
    //     gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    //     gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    //     gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
    //     if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
    //         pthread_mutex_lock(&mutex);
    //         waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
    //         //Reset the variables that are used for the D-controller.
    //         gps_lat_error_previous = 0;
    //         gps_lon_error_previous = 0;
    //         gps_lat_total_avarage = 0;
    //         gps_lon_total_avarage = 0;
    //         gps_rotating_mem_location = 0;
    //         //Reset the waypoints.
    //         l_lat_waypoint = 0;
    //         l_lon_waypoint = 0;
    //         pthread_mutex_unlock(&mutex);
    //     }
    // }
    
}


#endif //PATMOS_READ_GPS_H
