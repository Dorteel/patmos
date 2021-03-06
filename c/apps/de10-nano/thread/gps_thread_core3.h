//
// Created by rahul on 11/8/20.
//


#ifndef PATMOS_GPS_THREAD_H
#define PATMOS_GPS_THREAD_H

void gps_thread()
{   
    pthread_mutex_lock(&mutex);
    printf("Setting up GPS\n");
    pthread_mutex_unlock(&mutex);
    gps_setup();
    //__uint32_t timer = get_cpu_usecs();
    while(!program_off){
        // pthread_mutex_lock(&mutex);
        printf("zzz_CALL READ GPS_zzz\n");
        // pthread_mutex_unlock(&mutex);
        
        read_gps();

        pthread_mutex_lock(&mutex);
        printf("yyy_CALL END READ GPS_yyy\n");
        pthread_mutex_unlock(&mutex);
        
    
        //while (get_cpu_usecs() - timer < dt*1000000);
        // if(PRINT_COMMANDS)
        // {
            // pthread_mutex_lock(&mutex);
            //printf("GPS = %d , %d\n",lat_gps_actual, lon_gps_actual);
            // printf("gps loop_timer: %llu\n",get_cpu_usecs() - timer );
            // pthread_mutex_unlock(&mutex);
        // }
      //  timer = get_cpu_usecs();
    }
    printf("gps end\n");
}
#endif //PATMOS_GPS_THREAD_H
