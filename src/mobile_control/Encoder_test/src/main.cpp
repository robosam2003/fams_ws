#include <Arduino.h>

#include <EnableInterrupt.h>

#define M1PinA  A8
#define M1PinB  A9
#define M2PinA  A10
#define M2PinB  A11
#define M3PinA  A12
#define M3PinB  A13
#define M4PinA  A14
#define M4PinB  A15

volatile int M1encoderPos = 0;
volatile int M2encoderPos = 0;
volatile int M3encoderPos = 0;
volatile int M4encoderPos = 0;

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
  pinMode(M1PinA, INPUT);
  pinMode(M1PinB, INPUT);
  pinMode(M2PinA, INPUT);
  pinMode(M2PinB, INPUT);
  pinMode(M3PinA, INPUT);
  pinMode(M3PinB, INPUT);
  pinMode(M4PinA, INPUT);
  pinMode(M4PinB, INPUT);

  enableInterrupt(M1PinA, doM1EncoderA, CHANGE);
  //enableInterrupt(M1PinB, doM1EncoderB, RISING);
  enableInterrupt(M2PinA, doM2EncoderA, CHANGE);
  //enableInterrupt(M2PinB, doM2EncoderB, RISING);
  enableInterrupt(M3PinA, doM3EncoderA, CHANGE);
  //enableInterrupt(M3PinB, doM3EncoderB, RISING);
  enableInterrupt(M4PinA, doM4EncoderA, CHANGE);
  //enableInterrupt(M4PinB, doM4EncoderB, RISING);
  
  Serial.begin (9600);
}

void loop() {
  //Do stuff here
  Serial.print("|M1|");Padding(M1encoderPos);Serial.print   (M1encoderPos, DEC);
  Serial.print("|M2|");Padding(M2encoderPos);Serial.print   (M2encoderPos, DEC);
  Serial.print("|M3|");Padding(M3encoderPos);Serial.print   (M3encoderPos, DEC);
  Serial.print("|M4|");Padding(M4encoderPos);Serial.println (M4encoderPos, DEC);
}

void doM1EncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(M1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(M1PinB) == LOW) {
      M1encoderPos = M1encoderPos + 1;         // CW
    }
    else {
      M1encoderPos = M1encoderPos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(M1PinB) == HIGH) {
      M1encoderPos = M1encoderPos + 1;          // CW
    }
    else {
      M1encoderPos = M1encoderPos - 1;          // CCW
    }
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
    // check channel B to see which way encoder is turning
    if (digitalRead(M2PinB) == LOW) {
      M2encoderPos = M2encoderPos + 1;         // CW
    }
    else {
      M2encoderPos = M2encoderPos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(M2PinB) == HIGH) {
      M2encoderPos = M2encoderPos + 1;          // CW
    }
    else {
      M2encoderPos = M2encoderPos - 1;          // CCW
    }
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
    // check channel B to see which way encoder is turning
    if (digitalRead(M3PinB) == LOW) {
      M3encoderPos = M3encoderPos + 1;         // CW
    }
    else {
      M3encoderPos = M3encoderPos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(M3PinB) == HIGH) {
      M3encoderPos = M3encoderPos + 1;          // CW
    }
    else {
      M3encoderPos = M3encoderPos - 1;          // CCW
    }
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
    // check channel B to see which way encoder is turning
    if (digitalRead(M4PinB) == LOW) {
      M4encoderPos = M4encoderPos + 1;         // CW
    }
    else {
      M4encoderPos = M4encoderPos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(M4PinB) == HIGH) {
      M4encoderPos = M4encoderPos + 1;          // CW
    }
    else {
      M4encoderPos = M4encoderPos - 1;          // CCW
    }
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

void Padding(int encoder){
     if(encoder >= 0){
          if (encoder < 10){
          Serial.print("00");
          } else if(encoder < 100){
               Serial.print("0");
          }
     }
//     if(encoder < 0){
//          if (encoder > -10){
//          Serial.print("-00");
//          } else if(encoder > -100){
//               Serial.print("-0");
//          }
//     }
}