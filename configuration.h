#include <Arduino.h>
//PIN Declaration

//Receiver
int CH1 = 36; //channel 1
int CH2 = 38; //channel 2
int CH3 = 40; //channel 3
int CH4 = 42; //channel 4
int CH5 = 48; //channel 5
int CH6 = 49; //channel 6

//Motor Driver 1
int PWM1 = 13;  //PWM
int INA1 = 43;  //Motor direction inputA ("Clockwise")
int INB1 = 47;  //Motor direction inputB ("Anti-Clockwise")
int CS1 = A8;   //Current sense output

//Motor Driver 2
int PWM2 = 12;
int INA2 = 39;
int INB2 = 41;
int CS2 = A9;

//Motor Driver 3
int PWM3 = 10;
int INA3 = 35;
int INB3 = 37;
int CS3 = A10;

//Motor Driver 4
int PWM4 = 9;
int INA4 = 31;
int INB4 = 33;
int CS4 = A11;

//LED
int ledPin1 = 29;
int ledPin2 = 27;
int ledPin3 = 25;
int ledPin4 = 23;

//Test Switch
int buttonPin = 22;

//Servo Motor PWM
int servo1 = 44;  //Servo#1
int servo2 = 46;  //Servo#2
int servo3 = 45;  //Servo#3
int servo4 = 11;  //Servo#4
int gripper = 2; // Servo Gripper D2 (PWM)
int rotator = 34; // Servo for rotating gripper D3 (PWM)

//Stepper motor & driver
int pinDir = 4; //D4 direction
int pinStep = 5; //D5 step
int pinEnable = 6; //Enable
