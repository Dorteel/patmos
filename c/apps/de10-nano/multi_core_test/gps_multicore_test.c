#include "../Flight_controller_v2.h"
#include "../gps/read_gps.h"



void get_gps_data()
{
  int loop_counter = 0;
    //------ to filter data ------------------
    int str_temp[6];
    char str_tempc[6];
    int start_temp = 0;
    int end_temp = 6;
    bool b_temp = false;
    bool printed = false;
    bool equal_RMC = false;
    int start_c = 6;
    int end_c = 300;
    bool equal_VTG = false;


    millis(250);

    while (loop_counter < 500) {                                                           //Stay in this loop until the data variable data holds a q.
        if (loop_counter < 500)loop_counter++;
        millis(4);                                                              //Wait for 4000us to simulate a 250Hz loop.
        if (loop_counter == 1) {
            printf("\n");
            printf("====================================================================\n");
            printf("Checking gps data @ 9600bps\n");
            printf("====================================================================\n");
        }
        //if (loop_counter > 1 && loop_counter < 500){
        if (loop_counter >= 1 && loop_counter < 500) {
            while (uart2_read(&gps_data)) {
                printf("%c",gps_data);
                //The delimiter "$" is 36 in ASCII
                if (gps_data == 36) {
                    b_temp = true;
                }
                if (b_temp && (start_temp < end_temp)) {
                    str_temp[start_temp] = gps_data;
                    str_tempc[start_temp] = (char) gps_data;
                    start_temp++;
                }

                if (equal_RMC && (start_c < end_c)) {
                    str_c[start_c] = (char) gps_data;
                    start_c++;
                }
                //find the RMC string
                if ((start_temp == end_temp) && !equal_RMC) {//&&!printed){
                    //printed = true;
                    b_temp = false;
                    int comp = 0;
                    for (int j = 0; j < 6; j++) {
                        comp = comp + str_tempc[j] - cRMC[j];
                        str_c[j] = str_tempc[j];
                    }
                    if (comp == 0) {
                        equal_RMC = true;
                    }
                    start_temp = 0; // Try again?
                }

                //find the VTG string
                if ((start_temp == end_temp) && equal_RMC && !equal_VTG) {
                    b_temp = false;
                    int comp = 0;
                    for (int j = 0; j < 6; j++) {
                        comp = comp + str_tempc[j] - cVTG[j];
                        //str_c[j] = str_tempc[j];
                    }
                    if (comp == 0) {
                        equal_VTG = true;
                        //  printf("\n\n");
                    } else {
                        equal_RMC = false; //If the next string is not VTG, it must go back false
                        start_c = 6;
                    }
                    start_temp = 0; // Try again?
                }
            }
        }
    }
    result = gps_decode(&tpv, str_c);
    if (result != GPS_OK)
    {
        fprintf(stderr, "Error (%d): %s\n", result, gps_error_string(result));
        //  return EXIT_FAILURE;
    }

    lat_gps_actual = abs((double)tpv.latitude/GPS_LAT_LON_FACTOR);
    lon_gps_actual = abs((double)tpv.longitude/GPS_LAT_LON_FACTOR);
    double alt =  (double)tpv.altitude/GPS_VALUE_FACTOR;

    if (tpv.latitude >0)latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
    else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

    if (tpv.longitude >0)longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
    else longiude_east = 0;


    printf("latitude: %d longitude: %d altitude: %f \n",lat_gps_actual,lon_gps_actual,alt);

    printf("latitude_north: %d, longitude_east: %d\n",latitude_north,longiude_east);
}


int main(int argc, char **argv)
{
  gps_setup();
  for (int i = 0; i < 5; ++i)
  {
    /* code */
    get_gps_data();
  }

  return 0;
}
