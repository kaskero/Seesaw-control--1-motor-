#include <LSM303.h>
#include <L3G.h>
#include <IMU_lib.h>
#include <CF_lib.h>
#include <PID_v1.h>
#include <dogm_7036.h>
#include <Balancin_lib.h>

LSM303 accel;
L3G gyro;
IMU_lib imu_lib;
CF_lib cf_lib;

int* accel_data;
int* gyro_data;
int* noise_data;

dogm_7036 DOG;

double Setpoint, Input, Output;
double Kp=0, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double outMin = 0;
double outMax = 1000;
unsigned long SampleTime = 10; // ms

const int encoderPinA = 2;
const int encoderPinB = 3;
const int encoderButtonPin = 4; //push button switch

volatile int lastEncoded = 0;
volatile double encoderValue = 0;
volatile bool flag = false;

int aux = 0;

const unsigned int icr1 = 1000;
const unsigned int ocr1a = 100;

const int DIR = 8;

void setup() {
   // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setup()");

  Output = TIMER1_init(icr1, ocr1a);
  
  IMU_init(&imu_lib, &accel, &gyro);
  accel_data = imu_lib.Read_Accel();
  gyro_data = imu_lib.Read_Gyro();
  noise_data = imu_lib.Get_Noise();
  cf_lib.begin(accel_data);
  Input = cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
  
  LCD_init(&DOG, Kp, Ki, Kd, Setpoint, Input, Output);
  
  Setpoint = 0;
  PID_init(&myPID, SampleTime, outMin, outMax);

  pinMode(encoderPinA, INPUT_PULLUP); 
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderButtonPin, INPUT_PULLUP);

  // interrupt 0 --> pin 2
  // interrupt 1 --> pin 3 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  pinMode(9, OUTPUT); //OC1A
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH);

  while(aux<5)
    Param();
  if(aux>4) { 
    aux=0;
    DOG.cursor_onoff(false);
  }
}

void loop() {
  Param();
  if(aux>4) { 
    aux=0;
    DOG.cursor_onoff(false);
  }
  
  accel_data = imu_lib.Read_Accel();
  gyro_data = imu_lib.Read_Gyro();
  noise_data = imu_lib.Get_Noise();
  Input = cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
  Serial.println(Input);
  
  myPID.Compute();
  OCR1A = Output;
  IO_update(&DOG, Input, Output, aux);
}

void Param() {
  if(!digitalRead(encoderButtonPin)) 
    Param_select(&DOG, Setpoint, Kp, Ki, Kd, &encoderValue, &aux);
  
  if(flag) {
    Param_update(&DOG, &Setpoint, &Kp, &Ki, &Kd, encoderValue, aux);
    myPID.SetTunings(Kp, Ki, Kd);
    flag = false;
  }
}

// ISR de interrupts 0 y 1
void updateEncoder() {
  int MSB = digitalRead(encoderPinA); //MSB = most significant bit
  int LSB = digitalRead(encoderPinB); //LSB = least significant bit
  
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderValue += 0.125f;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue -= 0.125f;

  lastEncoded = encoded; //store this value for next time

  flag = true;
}


