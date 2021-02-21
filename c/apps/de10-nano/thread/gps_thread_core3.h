//
// Created by rahul on 11/8/20.
//


#ifndef PATMOS_GPS_THREAD_H
#define PATMOS_GPS_THREAD_H

void gps_thread()
{
    gps_setup();
    //Read the MPU-6050
    __uint32_t timer = get_cpu_usecs();
    while(!program_off){
        pthread_mutex_lock(&mutex);
        pthread_mutex_unlock(&mutex);
        read_gps();

        while (get_cpu_usecs() - timer < dt*1000000);
        // if(PRINT_COMMANDS)
        // {
        //     pthread_mutex_lock(&mutex);
        //     printf("gps loop_timer: %llu\n",get_cpu_usecs() - timer );
        //     pthread_mutex_unlock(&mutex);
        // }
        timer = get_cpu_usecs();
    }

}
#endif //PATMOS_GPS_THREAD_H
