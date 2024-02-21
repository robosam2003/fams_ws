#include <Arduino.h>

// Arduino **MEGA** for optical rotary encoder; plus an encoder simulator  Sept 2022 
// WOKWI animation : https://wokwi.com/projects/343001355413094995


/*
The first part of the program receives the quadrature signal from an optical rotary encoder 
and displays its position on the serial monitor. 

---------------------------------------------
If you have a 5 Volts encoder and an Arduino:
---------------------------------------------
Connect the outputs A and B of the 5 Volt encoder to the input pins 2 and 3 of the Arduino
(put resistors of 1K max in series between the pins and the wires A and B, for safety). 
Do not make the connections on pins 11 and 12, nor the 2 buttons on A0 and A3. 
You can delete the second part of the program, which is not used with a real encoder.

------------------------------------
If you don't have the real encoder :
------------------------------------
the second part of the program allows to simulate the quadrature function on pins 11 and 12
(these pins work like an encoder, they are controlled by the buttons).

With a real Arduino:
-------------------
Place resistors of 1K max as shown on the Wokwi diagram, to protect the pins from a programming error: 
output 12 to input 2, and output 11 to input 3; 
and also on the buttons, 1K max between A0 and the button, 1K max between A3 and the other button.
ATTENTION don't forget to set (prevmillis > 20) in the "additionalButtons()" function
(for Wokwi the value is 1, otherwise the reaction time of the buttons is too long).

In simulation with Wokwi:
-------------------------
The resistors are intentionally not connected, it is to indicate how to do with the real Arduino.
In case of blocking, it is sometimes necessary to stop the simulation, and restart it.
You may have noticed that English is not my usual language ;-)
*/


void ExtInt();

// ENCODER function : quadrature signal decoded with interrupt inputs 2 & 3 ----------------------------

  #define INPUT_ENCOD_A    48                                 // *** MEGA PIN2 (***=adapt with input pin)
  #define INPUT_ENCOD_B    46                                 // *** MEGA PIN3
  #define SignalA          B00010000                          // *** MEGA PIN2 = PORT_E  bit4 = B00010000
  #define SignalB          B00100000                          // *** MEGA PIN3 = PORT_E  bit5 = B00100000
  #define SignalAB         B00110000                          // *** both signals
  volatile int             ISRencodPos;                       // encoder position
  int                      encodLastPos;                      // previous position
  byte                     LastPort8 = SignalA;               // previous A/B state 


void setup(void) {                                            //
  pinMode(INPUT_ENCOD_A, INPUT_PULLUP);                       //
  pinMode(INPUT_ENCOD_B, INPUT_PULLUP);                       //
  attachInterrupt(digitalPinToInterrupt(INPUT_ENCOD_A), ExtInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INPUT_ENCOD_B), ExtInt, CHANGE);
  Serial.begin(115200);                                       // fast fast fast !
  Serial.println(F("waiting for encoder rotation..."));       // 
  //additionalSetup();   //!!!//                                // (!!! not necessary with real encoder)
}                                                             //


void ExtInt() {                                               /// OPTICAL ENCODER ext interrupt pin X, Y
     byte Port8  =  PINE & SignalAB;                          // *** PINE (PORT INPUT E)  ***for Mega***
      LastPort8 ^=  Port8;                                    //                                      
  if (LastPort8 & SignalA)   ISRencodPos++;                   // Rotation -> {ISRencodPos++; Sense = 1;}
  if (LastPort8 & SignalB)   ISRencodPos--;                   // Rotation <- {ISRencodPos--; Sense = 0;}
  if (    Port8 && (Port8 != SignalAB)) Port8 ^= SignalAB;    //                              (swap A-B)
      LastPort8  =  Port8;                                    //                  mieux vaut faire court
}                                                             //


void loop(void) {                                             /// MAIN LOOP
  noInterrupts();                                             //
    int encodPosition = ISRencodPos;                          //
  interrupts();                                               //

  if (encodLastPos != encodPosition) {                        // when the encoder change,
    encodLastPos = encodPosition;                             //
    Serial.println(encodPosition);                            // print encoder position
  }                                                           //
  //additionalButtons();  //!!!//                               // (!!! not necessary with real encoder)
}                                                             //
// END of the REAL ENCODER minimum function ----------------- // END REAL ROTARY ENCODER ---------------

/*
      |
      v


      |
      v


      |
      v 

*/ 

// EXTRA aditional function : BUTTONS & ENCODER SIMULATION---------------------------------------------
  
#define Phase1     B00100000                                  // SIMUL_ENCODER  bit 6 & 5 of Mega PORTB
#define Phase2     B01000000                                  // SIMUL_ENCODER  for direct PORT write,
#define Phase1_2   B01100000                                  // SIMUL_ENCODER  but not used
byte               Quadra = Phase1;                           // SIMUL_ENCODER  (digitalWrite used)


#define INPUT_buttonLeft    A0                                // PIN
#define INPUT_buttonRight   A3                                // PIN
#define OUTPUT_simulEncodA  12                                // PIN
#define OUTPUT_simulEncodB  11                                // PIN


#define buttonLeft      B00000001                             // BUTTON   PF0   (A0)   ***for Mega***
#define buttonRight     B00001000                             // BUTTON   PF3   (A3)   ***for Mega***
#define buttonAll       B00001001                             // BUTTON   
#define maskAutoRepeat  B00001001                             // BUTTON   auto-repeat define here concerned keys 
#define tempoRepeatSlow 60                                    // BUTTON   auto-repeat first tempo (multiple 20ms)
#define tempoRepeatFast  6                                    // BUTTON   auto-repeat second tempo(multiple 20ms)
unsigned long     prevmillis;                                 // BUTTONS  debounce period
byte              actionStat;                                 // BUTTONS  state    buttons
byte              actionDown;                                 // BUTTONS  pressed  buttons
byte              actionUp;                                   // BUTTONS  released buttons
byte              actionMemo;                                 // BUTTONS  memorised states buttons
byte              tempoAutoRepeat = tempoRepeatSlow;          // BUTTONS  auto-repeat double-timer



void additionalSetup(){                                       // setup
  pinMode(OUTPUT_simulEncodA, OUTPUT);                        // SIMUL_ENCODER
  pinMode(OUTPUT_simulEncodB, OUTPUT);                        // SIMUL_ENCODER
  pinMode(INPUT_buttonLeft,  INPUT_PULLUP);                   // BUTTON
  pinMode(INPUT_buttonRight, INPUT_PULLUP);                   // BUTTON
  digitalWrite(OUTPUT_simulEncodA,  Quadra & Phase2);         // initialise quadrature signal
  digitalWrite(OUTPUT_simulEncodB,  Quadra & Phase1);         // on pins 11, 12 
}                                                             // 


void additionalButtons(){                                     // 1 .. 8 BUTTONS, 16 with word instead byte
  if (millis() - prevmillis > 20) {    //! ATTENTION //       // Wokwi=1,real Mega=20 (ms debounce period)
    prevmillis = millis();                                    //
    byte iii   = ~PINF & buttonAll;                           // PORTF bit 0 & 3  (A0  A3)  ***for Mega***
    byte jjj   = actionStat;                                  //    
    actionStat = iii & actionMemo;                            // buttons status                 (state)
    actionDown = (jjj ^ actionStat) & actionStat;             // buttons is (are) just pressed  (front)
    actionUp   = (iii ^ jjj) & jjj;                           // buttons is (are) just released (front)
    actionMemo = iii;                                         // buttons configuration stored for next use

    if (actionStat & maskAutoRepeat) {                        // autoRepeat only on buttons in the mask
      tempoAutoRepeat--;                                      // 
      if (tempoAutoRepeat == 0) {                             // 
        tempoAutoRepeat = tempoRepeatFast;                    // autorepeat short tempo after 1st stroke
        actionDown = actionStat;                              // re-triggs buttons that are kept pressed
    } } else tempoAutoRepeat = tempoRepeatSlow;               // autoRepeat long tempo for first stroke


    
    if (buttonLeft & actionDown) {                            // ENCODER SIMULATION 
      if (Quadra && (Quadra != Phase1_2)) Quadra ^= Phase2;   // left button pressed
      else                                Quadra ^= Phase1;   // make quadrature signal
      digitalWrite(OUTPUT_simulEncodA,    Quadra &  Phase2);  // write quadrature signal
      digitalWrite(OUTPUT_simulEncodB,    Quadra &  Phase1);  // on pin 11,12
    }                                                         //
    
    if (buttonRight & actionDown) {                           // right button pressed
      if (Quadra && (Quadra != Phase1_2)) Quadra ^= Phase1;   // make quadrature signal
      else                                Quadra ^= Phase2;   //
      digitalWrite(OUTPUT_simulEncodA,    Quadra &  Phase2);  // write quadrature signal
      digitalWrite(OUTPUT_simulEncodB,    Quadra &  Phase1);  // on pin 11,12
    }                                                         //
  }                                                           //
}                                                             //