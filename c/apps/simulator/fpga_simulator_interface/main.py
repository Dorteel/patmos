import sim
import math
import sys
from time import sleep

class Client:
    def __enter__(self):
        self.intSignalName='legacyRemoteApiStepCounter'
        self.stepCounter=0
        self.maxForce=100
        sim.simxFinish(-1) # just in case, close all opened connections
        self.id=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
        return self
    
    def __exit__(self,*err):
        sim.simxFinish(-1)

with Client() as drone:
    print("running")

    if drone.id!=-1:
        print ('Connected to remote API server')        
         # Start streaming drone.intSignalName integer signal, that signals when a step is finished:
        sim.simxGetIntegerSignal(drone.id,drone.intSignalName,sim.simx_opmode_streaming)

        # ==================== Initialization ====================================
        # Syscall init()
        print('Getting objects')
        # Motors
        res,drone.m1=sim.simxGetObjectHandle(drone.id,'m1',sim.simx_opmode_blocking)
        res,drone.m2=sim.simxGetObjectHandle(drone.id,'m2',sim.simx_opmode_blocking)
        res,drone.m3=sim.simxGetObjectHandle(drone.id,'m3',sim.simx_opmode_blocking)
        res,drone.m4=sim.simxGetObjectHandle(drone.id,'m4',sim.simx_opmode_blocking)
        # Target velocities
        value = 20
        m1_t = -value
        m2_t = value 
        m3_t = -value 
        m4_t = value

        # Sensors
        res,drone.imu=sim.simxGetObjectHandle(drone.id,'IMU_fr',sim.simx_opmode_blocking)
        res,drone.baro=sim.simxGetObjectHandle(drone.id,'baro_fr',sim.simx_opmode_blocking)
        res,drone.gps=sim.simxGetObjectHandle(drone.id,'GPS_fr',sim.simx_opmode_blocking)
        imu_XYZ_old = [0, 0, 0]
        imu_ABG_old = [0, 0, 0]
        baro_altitude = 0
        compass = 0

        # Others
        centerVel_XYZ = [0, 0, 0]
        centerVel_XYZ_old = [0, 0, 0]
        centerAcc_XYZ = [0, 0, 0]
        centerVel_ABG = [0, 0, 0]
        centerVel_ABG_old = [0, 0, 0]
        centerAcc_ABG = [0, 0, 0]

        # ==================== main loop ====================================
        # ---> sysCall_actuation() <---
        sim.simxSetJointTargetVelocity(drone.id, drone.m1, m1_t, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(drone.id, drone.m2, m2_t, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(drone.id, drone.m3, m3_t, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(drone.id, drone.m4, m4_t, sim.simx_opmode_oneshot)

        # ---> sysCall_sensing()  <---
        res,imu_XYZ_value=sim.simxGetObjectPosition(drone.id,drone.imu,-1,sim.simx_opmode_blocking)
        res,imu_ABG_value = sim.simxGetObjectOrientation(drone.id,drone.imu,-1,sim.simx_opmode_blocking)
        res,baro_value = sim.simxGetObjectPosition(drone.id,drone.baro,-1,sim.simx_opmode_blocking)
        res,gps_value = sim.simxGetObjectOrientation(drone.id,drone.gps,-1,sim.simx_opmode_blocking)

        # Get relevant values 
        baro_altitude = baro_value[2]
        compass = gps_value[2]
    
        # Calculate velocity and acceleration, linear + angular
        for i in range(3):
            centerVel_XYZ[i] = imu_XYZ_value[i] - imu_XYZ_old[i]
            centerAcc_XYZ[i] = centerVel_XYZ[i] - centerVel_XYZ_old[i]
            centerVel_ABG[i] = imu_ABG_value[i] - imu_ABG_old[i]
            centerAcc_ABG[i] = centerVel_ABG[i] - centerVel_ABG_old[i]

        # Update
        imu_XYZ_old = imu_XYZ_value
        imu_ABG_old = imu_ABG_value
        centerVel_XYZ_old = centerVel_XYZ
        centerVel_ABG_old = centerVel_ABG
        
        sleep(10)
