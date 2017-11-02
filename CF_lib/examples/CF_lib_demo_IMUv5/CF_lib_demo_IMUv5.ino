#include <LSM6.h>
#include <IMU.h>
#include <CF_lib.h>

LSM6 imu;
IMU imu_lib;
CF_lib cf_lib;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println("enter setup()");

  imu_lib.begin(&imu);
  delay(100);
  imu_lib.getOffsetNoise();
  delay(100);
  
  int* accel_data = imu_lib.read_Acc();
  cf_lib.begin(accel_data);
  Serial.println("exit setup()");
}

void loop() {
  // put your main code here, to run repeatedly:
  int* accel_data = imu_lib.read_Acc();
  int* gyro_data = imu_lib.read_Gyro();
  int* noise_data = imu_lib.Get_Noise();

  float pitch = cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
}