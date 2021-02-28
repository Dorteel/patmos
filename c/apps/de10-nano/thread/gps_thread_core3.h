//
// Created by rahul on 11/8/20.
//


#ifndef PATMOS_GPS_THREAD_H
#define PATMOS_GPS_THREAD_H

void gps_thread()
{
    printf("Setting up GPS\n");
    gps_setup();
    //__uint32_t timer = get_cpu_usecs();
    while(!program_off){
        pthread_mutex_lock(&mutex);
        pthread_mutex_unlock(&mutex);
        printf("zzz_CALL READ GPS_zzz\n");
        read_gps();

        printf("yyy_CALL END READ GPS_yyy\n");

        //while (get_cpu_usecs() - timer < dt*1000000);
        if(PRINT_COMMANDS)
        {
            pthread_mutex_lock(&mutex);
            //printf("GPS = %d , %d\n",lat_gps_actual, lon_gps_actual);
            // printf("gps loop_timer: %llu\n",get_cpu_usecs() - timer );
            pthread_mutex_unlock(&mutex);
        }
      //  timer = get_cpu_usecs();
    }

}
#endif //PATMOS_GPS_THREAD_H
