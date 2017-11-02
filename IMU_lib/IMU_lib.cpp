/*
  IMU_lib.h
  Created by Oskar Casquero, March 28, 2017.
  Released into the public domain.
*/

#include "IMU_lib.h"
#include "Wire.h"
#include <LSM303.h>
#include <L3G.h>

IMU_lib::IMU_lib() {
}

void IMU_lib::beginAccel(LSM303* accel) {
	Wire.begin(); 
	
	_accel = accel;
	
	if (!_accel->init()) {
		Serial.println("Failed to autodetect accel type!");
		while (1);
	}
	_accel->enableDefault();

	// Pololu AltIMU-10 v4 breakout board
	// 4 g scale: xx00 1xxx
	byte lsm303_ctrl2 = _accel->readReg(LSM303::CTRL2);
	lsm303_ctrl2 |= _BV(3);
	lsm303_ctrl2 &= ~(_BV(5) | _BV(4));
	_accel->writeAccReg(LSM303::CTRL2, lsm303_ctrl2);
	
	Serial.println("IMU_lib accel 4g");
}

void IMU_lib::beginGyro(L3G* gyro) {
	Wire.begin(); 
	
	_gyro = gyro;
	
	if (!_gyro->init()) {
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}
	_gyro->enableDefault();

	// 500 dps scale: xx01 xxxx
	byte lgd20h_ctrl4 = _gyro->readReg(L3G::CTRL4);
	lgd20h_ctrl4 |= _BV(4);
	lgd20h_ctrl4 &= ~(_BV(5));
	_gyro->writeReg(L3G::CTRL4, lgd20h_ctrl4); 

	Serial.println("IMU_lib gyro 500 dps");
}

void IMU_lib::getOffsetNoise() {
	int sampleNum = 100;

	for(int i=0; i<sampleNum; i++) {
		Read_Accel();
		Read_Gyro();
		for(int j=0; j<6; j++) {
			data_offset[j] += data[j];
		}
		delay(10);
	}
		
	for(int j=0; j<6; j++) 
		data_offset[j] = data_offset[j]/sampleNum;  

	for(int j=0; j<sampleNum; j++) {  
		Read_Accel();
		Read_Gyro();
		for(int j=0; j<6; j++) {  
			int noise = abs(data[j] - data_offset[j]);  
			if(noise > data_noise[j])
				data_noise[j] = noise;      
		}
		delay(10);
	}
}

int* IMU_lib::Read_Accel() {
	_accel->readAcc();

	data[0] = _accel->a.x; 
	data[1] = _accel->a.y;
	data[2] = _accel->a.z;

	int a_x = data[0] - data_offset[0];
	int a_y = data[1] - data_offset[1];
	int a_z = data[2] - data_offset[2];
	
	//Low Pass Filter
	float alpha = 0.75;
    accel_data[0] = (alpha * a_x) + ((1 - alpha) * accel_data[0]);
    accel_data[1] = (alpha * a_y) + ((1 - alpha) * accel_data[1]);
    accel_data[2] = (alpha * a_z) + ((1 - alpha) * accel_data[2]);
  
	return accel_data;
}

int* IMU_lib::Read_Gyro() {
	_gyro->read();

	data[3] = _gyro->g.x;
	data[4] = _gyro->g.y;
	data[5] = _gyro->g.z;

	gyro_data[0] = data[3] - data_offset[3];
	gyro_data[1] = data[4] - data_offset[4];
	gyro_data[2] = data[5] - data_offset[5];
  
	return gyro_data;
}

int* IMU_lib::Get_Noise() {
  
	return data_noise;
}


