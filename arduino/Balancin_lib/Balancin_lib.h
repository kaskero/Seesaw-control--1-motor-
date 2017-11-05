#include <LSM303.h>
#include <L3G.h>
#include <IMU_lib.h>
#include <dogm_7036.h>
#include <PID_v1.h>

unsigned int TIMER1_init(unsigned int icr1, unsigned int ocr1a);

void IMU_init(IMU_lib* imu_lib, LSM303* accel, L3G* gyro);

void LCD_init(dogm_7036* DOG, 
			  double Kp, double Ki, double Kd, 
			  double Setpoint, double Input, double Output);

void PID_init(PID* myPID, unsigned long SampleTime, double outMin, double outMax);

void Param_select(dogm_7036* DOG, double Setpoint, 
				  double Kp, double Ki, double Kd,
                  volatile double* encoderValue, int* aux);
				  
void Param_update(dogm_7036* DOG, double* Setpoint, 
				  double* Kp, double* Ki, double* Kd, 
                  volatile double encoderValue, int aux);

void IO_update(dogm_7036* DOG, double Input, double Output, int aux);
