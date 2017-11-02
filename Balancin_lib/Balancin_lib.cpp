#include <Balancin_lib.h>

unsigned int TIMER1_init(unsigned int icr1, unsigned int ocr1a) {
	// 16.11.1 TCCR1A – Timer/Counter1 Control Register A
	// COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
	// The COM1A1:0 and COM1B1:0 control the Output Compare pins (OC1A and OC1B) behavior.
	// Table 16-3. Compare Output Mode, Phase Correct and Phase and Frequency Correct PWM
	// COM1A1/COM1B1 COM1A0/COM1B0
	//       1             0        Clear OC1A/OC1B on Compare Match when upcounting.
	//                              Set OC1A/OC1B on Compare Match when downcounting.
	TCCR1A |= _BV(COM1A1);
	TCCR1A &= ~(_BV(COM1A0));
	// 16.11.2 TCCR1B – Timer/Counter1 Control Register B
	// X X - WGM13 WGM12 CS12 CS11 CS10
	// Table 16-4. Waveform Generation Mode Bit Description
	// WGM13 WGM12 WGM11 WGM10 Tiimer/Counter_Mode_of_Operation TOP
	// 1     0     0     0     PWM, Phase and Frequency Correct ICR1
	TCCR1B |= _BV(WGM13);
	TCCR1B &= ~(_BV(WGM12));
	TCCR1A &= (~(_BV(WGM11)) & ~(_BV(WGM10)));
	// Table 16-5. Clock Select Bit Description
	// CS12 CS11 CS10 Description
	// 0    1    0    clk/8
	TCCR1B |= _BV(CS11);
	TCCR1B &= (~(_BV(CS12)) & ~(_BV(CS10)));

	Serial.print("TCCR1A: "); Serial.println(TCCR1A, BIN);
	Serial.print("TCCR1B: "); Serial.println(TCCR1B, BIN);

	// t_BIT_TCNT1: 1 / (16*10^6 / 8) = 0.5us
	ICR1 = icr1;   // si icr1=1000
	               // t_PWM: 2 * (icr1 * 0.5us) = 1ms --> 1KHz
	OCR1A = ocr1a; // si ocr1a=100
				   // 2 * (100 * 0.5us) = 0.1ms
                   // hay que multiplicar x2 porque es phase correct
	Serial.print("ICR1: "); Serial.println(ICR1);
	Serial.print("OCR1A: "); Serial.println(OCR1A);
	
	return OCR1A;
}

void IMU_init(IMU_lib* imu_lib, LSM303* accel, L3G* gyro) {
	imu_lib->beginAccel(accel);
	imu_lib->beginGyro(gyro);
	delay(100);
	imu_lib->getOffsetNoise();
}

void LCD_init(dogm_7036* DOG, 
			  double Kp, double Ki, double Kd, 
			  double Setpoint, double Input, double Output) {
	DOG->initialize(7,0,0,6,5,1,DOGM163); //SS, 0, 0 = HW SPI, RS, 4 = RESET, 1 = 5V, EA DOGM163-A (=3 lines)
	DOG->displ_onoff(true);
	DOG->cursor_onoff(false);	
	DOG->contrast(0x05);
	DOG->clear_display();

	char c[5];
	dtostrf(Kp, 5, 1, c);
	DOG->position(1,1);             
	DOG->string("Kp="); DOG->string(c);			
	            
	dtostrf(Ki, 5, 1, c);
	DOG->position(1,2);             
	DOG->string("Ki="); DOG->string(c);
	
	dtostrf(Kd, 5, 1, c);
	DOG->position(1,3);             
	DOG->string("Kd="); DOG->string(c);
	
	dtostrf(Setpoint, 5, 1, c);
	DOG->position(10,1);             
	DOG->string("S="); DOG->string(c);
	
	dtostrf(Input, 5, 1, c);
	DOG->position(10,2);             
	DOG->string("I="); DOG->string(c);
	
	dtostrf(Output, 5, 0, c);
	DOG->position(10,3);             
	DOG->string("O="); DOG->string(c);
}

void PID_init(PID* myPID, unsigned long SampleTime, double outMin, double outMax) {
	myPID->SetControllerDirection(DIRECT);
	myPID->SetMode(AUTOMATIC);
	myPID->SetOutputLimits(outMin, outMax); 
	myPID->SetSampleTime(SampleTime);
}

void Param_select(dogm_7036* DOG, double Setpoint, 
				  double Kp, double Ki, double Kd,
                  volatile double* encoderValue, int* aux) {
	delay(150); // antirrebote
	
	(*aux)++;
	switch(*aux) {
		case 1:
			*encoderValue = Kp;
			DOG->cursor_onoff(true);
			DOG->position(1,1);
			break;
		case 2:
			*encoderValue = Ki;
			DOG->position(1,2);
			break;
		case 3:
			*encoderValue = Kd;
			DOG->position(1,3);
			break;
		case 4:
			*encoderValue = Setpoint;
			DOG->position(10,1);
			break;
		default:
			break;
	}
}

void Param_update(dogm_7036* DOG, double* Setpoint, 
				  double* Kp, double* Ki, double* Kd, 
                  volatile double encoderValue, int aux) {
	char c[5];
    
	switch(aux) {
		case 1:
			*Kp = encoderValue;
			dtostrf(*Kp, 5, 2, c);
			DOG->position(4,1);             
			DOG->string(c);
			DOG->position(1,1); 
			break;
		case 2:
			*Ki = encoderValue;
			dtostrf(*Ki, 5, 2, c);
			DOG->position(4,2);             
			DOG->string(c);
			DOG->position(1,2); 
			break;
		case 3:
			*Kd = encoderValue;
			dtostrf(*Kd, 5, 2, c);
			DOG->position(4,3);             
			DOG->string(c);
			DOG->position(1,3); 
			break;
		case 4:
			*Setpoint = encoderValue;
			dtostrf(*Setpoint, 5, 1, c);
			DOG->position(12,1);             
			DOG->string(c);
			DOG->position(10,1); 
			break;
		default:
			break;
	}
}

void IO_update(dogm_7036* DOG, double Input, double Output, int aux) {
	char c[5];

	dtostrf(Input, 5, 1, c);
	DOG->position(12,2);             
	DOG->string(c);
	
	dtostrf(Output, 5, 0, c);
	DOG->position(12,3);             
	DOG->string(c);
	
	switch(aux) {
		case 1:
			DOG->position(1,1); 
			break;
		case 2:
			DOG->position(1,2);
			break;
		case 3:
			DOG->position(1,3);
			break;
		case 4:
			DOG->position(10,1);
			break;
		default:
			break;
	}
}