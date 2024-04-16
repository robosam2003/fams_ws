// Started on: 12/02/24
// Last edited: 15/04/24
// Author: Tobias McNeil

/*
Function:

Interfaces between the mobile robot node and the mobile robot hardware in the FAMS system. 

Contains a speed controller to match a target provided by the mobile robot node.

Contains no unnecessary functions, not capable of exporting data for analysis.

*/


#include <Arduino.h>
#include <SPI.h>
#include <EnableInterrupt.h>

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

volatile int M1encoderPos = 0; //Count of encoder position
volatile int M2encoderPos = 0;
volatile int M3encoderPos = 0;
volatile int M4encoderPos = 0;

int pwm1, pwm2, pwm3, pwm4; // PWM signal to motor driver
int prevpwm1, prevpwm2, prevpwm3, prevpwm4; // Previous PWM for integration
int dir1, dir2, dir3, dir4;// Direction of motor sent to motor driver

long currT = 0;// Current time of system
float deltaT = 0;// Time taken to measure certain amount of encoder pings
volatile long prevT = 0;// Previous time measurement was taken at
volatile float velocity_i1 = 0, velocity_i2 = 0, velocity_i3 = 0, velocity_i4 = 0, left_vel = 0, right_vel = 0;// Velocities of wheels and target velocities from node
volatile int pos1, pos2, pos3, pos4;// Current count of encoder position
volatile int posprev1, posprev2, posprev3, posprev4;// Previous count of encoder position
float vt1, vt2, vt3, vt4, err1, err2, err3, err4, u1, u2, u3, u4;// Contorl variables

void motor();// Funciton eUpdates Motor parameters
void controlLoop();// Executes speed control for the mobile robot

void doM1EncoderA();// Executes on detection of an encoder Ping
void doM2EncoderA();
void doM3EncoderA();
void doM4EncoderA();

void setup() {
  unsigned int configWord;
  
  pinMode(M1PinA, INPUT);//Establishes encoder input pins
  pinMode(M2PinA, INPUT);
  pinMode(M3PinA, INPUT);
  pinMode(M4PinA, INPUT);

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
  dir1 = 1; dir2 = 1; dir3 = 1; dir4 = 1; // Set direction 1 is forward, 0 is reverse
  pwm1 = 0; pwm2 = 0; pwm3 = 0; pwm4 = 0; // Set speed (0-255)
  prevpwm1 = 0, prevpwm2 = 0, prevpwm3 = 0, prevpwm4 = 0;

  digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled

  enableInterrupt(M1PinA, doM1EncoderA, CHANGE);// Links encoder count functions to hardware interrupts
  enableInterrupt(M2PinA, doM2EncoderA, CHANGE);
  enableInterrupt(M3PinA, doM3EncoderA, CHANGE);
  enableInterrupt(M4PinA, doM4EncoderA, CHANGE);
}

void loop() {
  dir1 = 1; dir2 = 2; dir3 = 1; dir4 = 1;// Sets all motors to forward
  left_vel = Serial.read();// Read diff drive wheel velocities from node
  right_vel = Serial.read();
  vt1 = vt2 = right_vel;// Set diff drive wheel velocities to control loop targets
  vt3 = vt4 = left_vel;
  controlLoop();// Update control loop
  motor();// Update motor parameters
}

void motor(){
  digitalWrite(DIR_M1, dir1);
  analogWrite(PWM_M1, pwm1);

	digitalWrite(DIR_M2, dir2);
	analogWrite(PWM_M2, pwm2);

	digitalWrite(DIR_M3, dir3);
	analogWrite(PWM_M3, pwm3);

	digitalWrite(DIR_M4, dir4);
	analogWrite(PWM_M4, pwm4);
}

void controlLoop(){
  // Retireving current encoder positions
  noInterrupts();
  pos1 = M1encoderPos;
  pos2 = M2encoderPos;
  pos3 = M3encoderPos;
  pos4 = M4encoderPos;
  interrupts();
  // Creates a time value to be referenced as the window of measurement
  currT = micros();
  deltaT = ((float) (currT-prevT))/1.0e6;// Calculates window of measurement in seconds
  velocity_i1 = (pos1 - posprev1)/deltaT;// Calculates difference in value of current and prev encoder pos as a funciton of time
  velocity_i2 = (pos2 - posprev2)/deltaT;// In units of encoder pings per second
  velocity_i3 = (pos3 - posprev3)/deltaT;
  velocity_i4 = (pos4 - posprev4)/deltaT;
  
  posprev1 = pos1;// Sets new previous encoder pos
  posprev2 = pos2;
  posprev3 = pos3;
  posprev4 = pos4;
  prevT = currT;// Sets new previous time
  
  float scl_fctr = 10.625;// Adjust if speed behaviour is odd, matches ranges of rad/s to PWM
  float Kp = 0.5*scl_fctr;// Adjust if control behaviour erratic/oscillatory
  
  err1 = vt1 - velocity_i1*(768/2*PI);// Sets to units of rad/s
  err2 = vt2 - velocity_i2*(768/2*PI);// Generates error from target velocity
  err3 = vt3 - velocity_i3*(768/2*PI);
  err4 = vt4 - velocity_i4*(768/2*PI);
  
  u1 = (Kp*err1);// Converts from rad/s to PWM
  u2 = (Kp*err2);
  u3 = (Kp*err3);
  u4 = (Kp*err4);
  
  int p1 = u1 + 0.5, p2 = u2 + 0.5, p3 = u3 + 0.5, p4 = u4 + 0.5;// Adds buffer so taht float to int will round correctly
  pwm1 = prevpwm1 + p1;// Integrates the control signal
  pwm2 = prevpwm2 + p2;
  pwm3 = prevpwm3 + p3;
  pwm4 = prevpwm4 + p4;
  
  //Bounds PWM to prevent any escalating positive or negative out of range values
  if(pwm1>255){
    pwm1 = 255;
  }
  else if(pwm1 <= 0){
    pwm1 = 0;
  }
  if(pwm2>255){
    pwm2 = 255;
  }
  else if(pwm2 <= 0){
    pwm2 = 0;
  }
  if(pwm3>255){
    pwm3 = 255;
  }
  else if(pwm3 <= 0){
    pwm3 = 0;
  }
  if(pwm4>255){
    pwm4 = 255;
  }
  else if(pwm4 <= 0){
    pwm4 = 0;
  }

  prevpwm1 = pwm1;// Sets previous PWM for next loop
  prevpwm2 = pwm2; 
  prevpwm3 = pwm3; 
  prevpwm4 = pwm4;

}

void doM1EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M1PinA) == HIGH) {
    M1encoderPos = M1encoderPos + 1;// If a encoder ping is detected, code is interrupted and the coutn increased before code restarts
  }
}

void doM2EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M2PinA) == HIGH) {
    M2encoderPos = M2encoderPos + 1;// If a encoder ping is detected, code is interrupted and the coutn increased before code restarts
  }
  }
}

void doM3EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M3PinA) == HIGH) {
    M3encoderPos = M3encoderPos + 1;// If a encoder ping is detected, code is interrupted and the coutn increased before code restarts
  }
  }
}

void doM4EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M4PinA) == HIGH) {
    M4encoderPos = M4encoderPos + 1;// If a encoder ping is detected, code is interrupted and the coutn increased before code restarts
  }
  }
}