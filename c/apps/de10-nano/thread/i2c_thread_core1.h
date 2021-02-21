//
// Created by rahul on 11/8/20.
//

#ifndef PATMOS_I2C_THREAD_H
#define PATMOS_I2C_THREAD_H

void i2c_thread()
{
    gyro_setup();
    setup_compass();
    read_compass();                                               //Read and calculate the compass data.
    pthread_mutex_lock(&mutex);
    angle_yaw = actual_compass_heading;                           //Set the initial compass heading.
    pthread_mutex_unlock(&mutex);
    barometer_setup();
    if(GYRO_CALLIB)callibrate_gyro();
    //Read the MPU-6050
    __uint32_t timer = get_cpu_usecs();
    while(!program_off){
        //Read the MPU-6050, barmeter and compass
        pthread_mutex_lock(&mutex);
        pthread_mutex_unlock(&mutex);
        if(compass_calibration_on==1)
        {
            callibrate_compass();
        }
        if(level_calibration_on==1)
        {
            callibrate_level();
        }
        read_compass();
        gyro_signalen();
        read_barometer();
        while (get_cpu_usecs() - timer < dt*1000000);
        if(PRINT_COMMANDS)
        {
            pthread_mutex_lock(&mutex);
            printf("i2c loop_timer: %llu\n",get_cpu_usecs() - timer );
            pthread_mutex_unlock(&mutex);
        }
        timer = get_cpu_usecs();
    }                                                           //Invert acc_z.

}

#endif //PATMOS_I2C_THREAD_H
