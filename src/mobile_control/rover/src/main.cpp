/*
Author: Tobias McNeil
Started: 19/02/24
Last Edited: 21/02/24

Purpose: To control 4WD mecanum robots. Contains a test function to check direction of motor hookups
Contains a basic wasdqe control for orthogonal and rotation control via the keyboard
Should contain funciton to take input of an x and y and then run the motors to it's best guess of where that is.
May have encoder functionality later

19/02/24: Successful hookup of motors
20/02/24: Successful control via serial monitor of Arduino and ROS
*/


#include <Arduino.h>
// include the SPI library:
#include <SPI.h>
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

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

int pwm1, pwm2, pwm3, pwm4, pwm;
boolean dir1, dir2, dir3, dir4, dir;

	// put function declarations here:
void motorContorl(char direction);

void motorTest();

void setup() {
	
  unsigned int configWord;

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
  Serial.begin(9600);
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
  dir1 = 0; dir2 = 0; dir3 = 0; dir4 = 0; // Set direction
  pwm1 = 0; pwm2 = 0; pwm3 = 0; pwm4 = 0;// Set speed (0-255)

digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled
} // End setup


void loop() {
	
    /*
    maths: Mecanum wheel diameter = 100mm
    262*359 arena size
    414*350*100
    so (0,0) frame is (207,175) into the robot's footspace
    Circumference = pi*100 mm = 314.15 ish
    so one rotation of wheels travels maximum 314 mm 

    estimate average slippage then create a rough file,
    then work on encoders. 
    */

   motorTest();

    //Keyboard control via Serial monitor
    /*
	if (Serial.available()){
		char direction = Serial.read();
		motorContorl(direction);
  }
	delay(100);
    */
}

	// put function definitions here:
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

void motorTest(){
	dir1 = 0; dir2 = 0; dir3 = 0; dir4 = 0; //0 = forwards, 1 = backwards
  	pwm1 = 100; pwm2 = 10; pwm3 = 250; pwm4 = 250;	
/*
	delay(1000);
 	digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1);
	delay(1000);
	dir1 = 1;
	digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1);

	delay(1000);
 	digitalWrite(DIR_M2, dir2);
    analogWrite(PWM_M2, pwm1);
	delay(1000);
	dir2 = 1;
	digitalWrite(DIR_M2, dir2);
    analogWrite(PWM_M2, pwm1);
*/
	//delay(1000);
 	digitalWrite(DIR_M3, dir3);
    analogWrite(PWM_M3, pwm1);
	/*delay(1000);
	dir3 = 1;
	digitalWrite(DIR_M3, dir3);
    analogWrite(PWM_M3, pwm1);
*/
	//delay(1000);
 	digitalWrite(DIR_M4, dir4);
    analogWrite(PWM_M4, pwm1);
	/*delay(1000);
	dir4 = 1;
	digitalWrite(DIR_M4, dir4);
    analogWrite(PWM_M4, pwm1);
    */
}