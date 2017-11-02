/*
  IMU_lib.h
  Created by Oskar Casquero, March 28, 2017.
  Released into the public domain.
*/

#ifndef IMU_lib_h
#define IMU_lib_h

#include "Arduino.h"
#include <LSM303.h>
#include <L3G.h>

class IMU_lib {
	public:
		IMU_lib();
    
		void beginAccel(LSM303* _accel);
		void beginGyro(L3G* _gyro);
		void getOffsetNoise();
	
		int* Read_Accel();
		int* Read_Gyro();
		int* Get_Noise();
	
	private:
		LSM303* _accel;
		L3G* _gyro;
	
		int accel_data[3] = {0, 0, 0};
		int gyro_data[3] = {0, 0, 0};;

		int data[6] = {0, 0, 0, 0, 0, 0};
		int data_offset[6] = {0, 0, 0, 0, 0, 0};
		int data_noise[6] = {0, 0, 0, 0, 0, 0};
};

#endif