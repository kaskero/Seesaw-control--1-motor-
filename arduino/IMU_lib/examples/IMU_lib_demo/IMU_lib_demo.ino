#include <LSM303.h>
#include <L3G.h>
#include <IMU_lib.h>

LSM303 accel;
L3G gyro;
IMU_lib imu_lib;
  
void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial.println("setup()");

	imu_lib.beginAccel(&accel);
	imu_lib.beginGyro(&gyro);
	delay(100);
	imu_lib.getOffsetNoise();
}

void loop() {
	// put your main code here, to run repeatedly:
	int* accel_data = imu_lib.Read_Accel();
	Serial.print("accel_x: "); Serial.println(*accel_data);
	Serial.print("accel_y: "); Serial.println(*(accel_data+1));
	Serial.print("accel_z: "); Serial.println(*(accel_data+2));
  
	int* gyro_data = imu_lib.Read_Gyro();
	Serial.print("gyro_x: "); Serial.println(*gyro_data);
	Serial.print("gyro_y: "); Serial.println(*(gyro_data+1));
	Serial.print("gyro_z: "); Serial.println(*(gyro_data+2));

	delay(1000);
}
