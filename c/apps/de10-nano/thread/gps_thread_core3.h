//
// Created by rahul on 11/8/20.
//


#ifndef PATMOS_GPS_THREAD_H
#define PATMOS_GPS_THREAD_H

void gps_thread()
{
    gps_setup();
    //Read the MPU-6050
    while(!program_off){
        pthread_mutex_lock(&mutex);
        pthread_mutex_unlock(&mutex);
        read_gps();
    }

}
#endif PATMOS_GPS_THREAD_H
