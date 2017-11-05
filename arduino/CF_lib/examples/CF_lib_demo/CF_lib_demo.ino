#include <LSM303.h>
#include <L3G.h>
#include <IMU_lib.h>
#include <CF_lib.h>

LSM303 accel;
L3G gyro;
IMU_lib imu_lib;
CF_lib cf_lib;
  
void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial.println("setup()");

	imu_lib.beginAccel(&accel);
	imu_lib.beginGyro(&gyro);
	delay(100);
	imu_lib.getOffsetNoise();
	
	int* accel_data = imu_lib.Read_Accel();
	cf_lib.begin(accel_data);
}

void loop() {
	// put your main code here, to run repeatedly:
	int* accel_data = imu_lib.Read_Accel();
	int* gyro_data = imu_lib.Read_Gyro();
	int* noise_data = imu_lib.Get_Noise();

	float pitch = cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
	Serial.print(" pitch: "); Serial.println(pitch);
}
