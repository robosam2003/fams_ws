#include <Arduino.h>
#include <SPI.h>
#include <EnableInterrupt.h>
#include <string.h>

// Encoder Pins
#define M1PinA  A8
#define M1PinB  A9
#define M2PinA  A10
#define M2PinB  A11
#define M3PinA  A12
#define M3PinB  A13
#define M4PinA  A14
#define M4PinB  A15
// L9958 slave select pins for SPI
#define SS_M4 14
#define SS_M3 13 
#define SS_M2 12
#define SS_M1 11
// L9958 DIRection pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7
// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5
#define PWM_M4 6     // Timer0

#define ENABLE_MOTORS 8

volatile int M1encoderPos = 0;
volatile int M2encoderPos = 0;
volatile int M3encoderPos = 0;
volatile int M4encoderPos = 0;

int pwm1, pwm2, pwm3, pwm4;
int prevpwm1, prevpwm2, prevpwm3, prevpwm4;
int dir1, dir2, dir3, dir4;

long currT = 0;
float deltaT = 0;
volatile float velocity_i1 = 0;  volatile float velocity_i2 = 0;  volatile float velocity_i3 = 0;  volatile float velocity_i4 = 0;
volatile long prevT = 0;
volatile int pos1, pos2, pos3, pos4;
volatile int posprev1, posprev2, posprev3, posprev4;
float vt, err1, err2, err3, err4, u1, u2, u3, u4;

char dataStr[35] = "";
char store1[8];char store2[8];char store3[8];char store4[8];

void motorContorl(char direction);

void doM1EncoderA();
void doM1EncoderB();
void doM2EncoderA();
void doM2EncoderB();
void doM3EncoderA();
void doM3EncoderB();
void doM4EncoderA();
void doM4EncoderB();
void Padding(int encoder);

void setup() {
  unsigned int configWord;
  
  pinMode(M1PinA, INPUT);
  pinMode(M1PinB, INPUT);
  pinMode(M2PinA, INPUT);
  pinMode(M2PinB, INPUT);
  pinMode(M3PinA, INPUT);
  pinMode(M3PinB, INPUT);
  pinMode(M4PinA, INPUT);
  pinMode(M4PinB, INPUT);

  /*
  |2|------|3|
    |   |  |		Hookup diagram of wheels
    |  \ / |
    --------
  |1|-------|4| 
  
  */

  // put your setup code here, to run once:
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, LOW);  // HIGH = not selected
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, LOW);
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, LOW);
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, LOW);

  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);

  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0

  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT); 
  digitalWrite(ENABLE_MOTORS, HIGH);  // HIGH = disabled
  Serial.begin(115200);
  /******* Set up L9958 chips *********
   ' L9958 Config Register
  ' Bit
  '0 - RES
  '1 - DR - reset
  '2 - CL_1 - curr limit
  '3 - CL_2 - curr_limit
  '4 - RES
  '5 - RES
  '6 - RES
  '7 - RES
  '8 - VSR - voltage slew rate (1 enables slew limit, 0 disables)
  '9 - ISR - current slew rate (1 enables slew limit, 0 disables)
  '10 - ISR_DIS - current slew disable
  '11 - OL_ON - open load enable
  '12 - RES
  '13 - RES
  '14 - 0 - always zero
  '15 - 0 - always zero
  */  // set to max current limit and disable ISR slew limiting
  configWord = 0b0000010000001100;

  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high

  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH);
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);

  //Set initial actuator settings to pull at 0 speed for safety
  dir1 = 0; dir2 = 1; dir3 = 0; dir4 = 1; // Set direction 1 is forward, 0 is reverse, 1 also is minus
  pwm1 = 0; pwm2 = 0; pwm3 = 0; pwm4 = 0; // Set speed (0-255)
  prevpwm1 = 0, prevpwm2 = 0, prevpwm3 = 0, prevpwm4 = 0;

  digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled

  enableInterrupt(M1PinA, doM1EncoderA, CHANGE);
  //enableInterrupt(M1PinB, doM1EncoderB, RISING);
  enableInterrupt(M2PinA, doM2EncoderA, CHANGE);
  //enableInterrupt(M2PinB, doM2EncoderB, RISING);
  enableInterrupt(M3PinA, doM3EncoderA, CHANGE);
  //enableInterrupt(M3PinB, doM3EncoderB, RISING);
  enableInterrupt(M4PinA, doM4EncoderA, CHANGE);
  //enableInterrupt(M4PinB, doM4EncoderB, RISING);

  Serial.print("M1_Velocity");Serial.print(",");
  Serial.print("M2_Velocity");Serial.print(",");
  Serial.print("M3_Velocity");Serial.print(",");
  Serial.println("M4_Velocity");
}

void loop() {
  //Output gear to encoder is a 1:64 gearbox
  //Wheel diameter is 100 mm
  dataStr[0] = 0;
  store1[0] = 0;store1[1] = 0;store1[2] = 0;store1[3] = 0;store1[4] = 0;store1[5] = 0;store1[6] = 0;
  store2[0] = 0;store2[1] = 0;store2[2] = 0;store2[3] = 0;store2[4] = 0;store2[5] = 0;store2[6] = 0;
  store3[0] = 0;store3[1] = 0;store3[2] = 0;store3[3] = 0;store3[4] = 0;store3[5] = 0;store3[6] = 0;
  store4[0] = 0;store4[1] = 0;store4[2] = 0;store4[3] = 0;store4[4] = 0;store4[5] = 0;store4[6] = 0;
  //Retireving current encoder positions
  noInterrupts();
  pos1 = M1encoderPos;
  pos2 = M2encoderPos;
  pos3 = M3encoderPos;
  pos4 = M4encoderPos;
  interrupts();
  //Creates a time value to be referenced as the window of measurement
  
  currT = micros();
  deltaT = ((float) (currT-prevT))/1.0e6;//Calculates window of measurement in seconds
  velocity_i1 = (pos1 - posprev1)/deltaT;//Calculates difference in value of current and prev encoder pos
  velocity_i2 = (pos2 - posprev2)/deltaT;//In units of pings per window
  velocity_i3 = (pos3 - posprev3)/deltaT;
  velocity_i4 = (pos4 - posprev4)/deltaT;
  
  posprev1 = pos1;//Sets new previous encoder pos
  posprev2 = pos2;
  posprev3 = pos3;
  posprev4 = pos4;
  prevT = currT;//Sets new previous time
  
  //pwm1 = 150, pwm2 = 150, pwm3 = 150, pwm4 = 150;

  vt = 1000;  //Target velocity, somewhere between 0 and 1500
  
  float Kp = 0.5;

  err1 = vt - velocity_i1;//Units of relative to current target pings per window
  err2 = vt - velocity_i2;
  err3 = vt - velocity_i3;
  err4 = vt - velocity_i4;
  
  float scl_fctr = 12.1568627;

  u1 = (Kp*err1)/scl_fctr;//Units of relative to current target pings per window
  u2 = (Kp*err2)/scl_fctr;
  u3 = (Kp*err3)/scl_fctr;
  u4 = (Kp*err4)/scl_fctr;
  
  int p1 = u1 + 0.5, p2 = u2 + 0.5, p3 = u3 + 0.5, p4 = u4 + 0.5;
  pwm1 = prevpwm1 + p1;
  pwm2 = prevpwm2 + p2;
  pwm3 = prevpwm3 + p3;
  pwm4 = prevpwm4 + p4;
  
  if(pwm1>255){
    pwm1 = 255;
  }
  if(pwm2>255){
    pwm2 = 255;
  }
  if(pwm3>255){
    pwm3 = 255;
  }
  if(pwm4>255){
    pwm4 = 255;
  }
  
  //pwm1 = 150, pwm2 = 150, pwm3 = 150, pwm4 = 150;

  digitalWrite(DIR_M1, dir1);
  analogWrite(PWM_M1, pwm1);

	digitalWrite(DIR_M2, dir2);
	analogWrite(PWM_M2, pwm2);

	digitalWrite(DIR_M3, dir3);
	analogWrite(PWM_M3, pwm3);

	digitalWrite(DIR_M4, dir4);
	analogWrite(PWM_M4, pwm4);

  prevpwm1 = pwm1;
  prevpwm2 = pwm2; 
  prevpwm3 = pwm3; 
  prevpwm4 = pwm4;
  
  Serial.print(velocity_i1);Serial.print(",");
  Serial.print(velocity_i2);Serial.print(",");
  Serial.print(velocity_i3);Serial.print(",");
  Serial.println(velocity_i4);
  
}


void motorContorl(char direction){
	
	switch (direction)
	{
	case 'w' :
		dir1 = 0; dir2 = 0; dir3 = 0; dir4 = 0; //0 = forwards, 1 = backwards
  		pwm1 = 150; pwm2 = 150; pwm3 = 150; pwm4 = 150;	

		break;
	case 's' :
		dir1 = 1; dir2 = 1; dir3 = 1; dir4 = 1; //0 = forwards, 1 = backwards
  		pwm1 = 150; pwm2 = 150; pwm3 = 150; pwm4 = 150;	

		break;
	case 'a' :
		dir1 = 0; dir2 = 1; dir3 = 0; dir4 = 1; //0 = forwards, 1 = backwards
  		pwm1 = 150; pwm2 = 150; pwm3 = 150; pwm4 = 150;	

		break;
	case 'd' :
		dir1 = 1; dir2 = 0; dir3 = 1; dir4 = 0; //0 = forwards, 1 = backwards
  		pwm1 = 150; pwm2 = 150; pwm3 = 150; pwm4 = 150;	

		break;
	case 'q' :
		dir1 = 0; dir2 = 0; dir3 = 1; dir4 = 1; //0 = forwards, 1 = backwards
  		pwm1 = 150; pwm2 = 150; pwm3 = 150; pwm4 = 150;	

		break;
	case 'e' :
		dir1 = 1; dir2 = 1; dir3 = 0; dir4 = 0; //0 = forwards, 1 = backwards
  		pwm1 = 150; pwm2 = 150; pwm3 = 150; pwm4 = 150;	

		break;
	case 'b' :
		pwm1 = 0; pwm2 = 0; pwm3 = 0; pwm4 = 0;
		break;
	default:
		
		break;
	}
	
	digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1);

	digitalWrite(DIR_M2, dir2);
	analogWrite(PWM_M2, pwm2);

	digitalWrite(DIR_M3, dir3);
	analogWrite(PWM_M3, pwm3);

	digitalWrite(DIR_M4, dir4);
	analogWrite(PWM_M4, pwm4);
}

void doM1EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M1PinA) == HIGH) {
    M1encoderPos = M1encoderPos + 1;
  }
}
void doM1EncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(M1PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(M1PinA) == HIGH) {
      M1encoderPos = M1encoderPos + 1;         // CW
    }
    else {
      M1encoderPos = M1encoderPos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(M1PinA) == LOW) {
      M1encoderPos = M1encoderPos + 1;          // CW
    }
    else {
      M1encoderPos = M1encoderPos - 1;          // CCW
    }
  }
}
void doM2EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M2PinA) == HIGH) {
    M2encoderPos = M2encoderPos + 1;
  }
}
void doM2EncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(M2PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(M2PinA) == HIGH) {
      M2encoderPos = M2encoderPos + 1;         // CW
    }
    else {
      M2encoderPos = M2encoderPos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(M2PinA) == LOW) {
      M2encoderPos = M2encoderPos + 1;          // CW
    }
    else {
      M2encoderPos = M2encoderPos - 1;          // CCW
    }
  }
}
void doM3EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M3PinA) == HIGH) {
    M3encoderPos = M3encoderPos + 1;
  }
}
void doM3EncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(M3PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(M3PinA) == HIGH) {
      M3encoderPos = M3encoderPos + 1;         // CW
    }
    else {
      M3encoderPos = M3encoderPos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(M3PinA) == LOW) {
      M3encoderPos = M3encoderPos + 1;          // CW
    }
    else {
      M3encoderPos = M3encoderPos - 1;          // CCW
    }
  }
}
void doM4EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M4PinA) == HIGH) {
    M4encoderPos = M4encoderPos + 1;
  }
}
void doM4EncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(M4PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(M4PinA) == HIGH) {
      M4encoderPos = M4encoderPos + 1;         // CW
    }
    else {
      M4encoderPos = M4encoderPos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(M4PinA) == LOW) {
      M4encoderPos = M4encoderPos + 1;          // CW
    }
    else {
      M4encoderPos = M4encoderPos - 1;          // CCW
    }
  }
}

