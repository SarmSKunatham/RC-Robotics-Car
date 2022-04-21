/* 
 * Set Board: Tools --> Board --> Arduino Mega2560
 * Set Processor: Tools --> Processor --> ATmega2560
*/


#include <Servo.h>  //servo library
#include "configuration.h" //from configuration file

// Mode
int mode = 1;
int Index=0;

//=============== Function Name Declaration ===============
  double Deadband(double value,double limit);
  int OutputToMotor1(int value);
  int OutputToMotor2(int value);
  int OutputToMotor3(int value);
  int OutputToMotor4(int value);
  

//=============== Parameters Declaration ==================

// Type Servo
  Servo myServo1;
  Servo myServo2;
  Servo myServo3;
  Servo myServo4;
  Servo myGripper;
  Servo myRotator;
// Time
  unsigned long previousLoopTime = 0; //unsigned for only positive value
  unsigned long loopTime = 0; //unsigned for only positive value
// Input Signal from Receiver 
  int input1 = 0;
  int input2 = 0;
  int input3 = 0;
  int input4 = 0;
  int input5 = 0;
  int input6 = 0;
// Output Signal to Motor Driver  
  int out1 = 0;
  int out2 = 0;
  int out3 = 0;
  int out4 = 0;
// Output Signal to Stepmotor
  int out5 =0;
// Processed Input Signal
  int left_w = 0;
  int right_w = 0;
// Motor's Current
float currentValue1 = 0.0;
float currentValue2 = 0.0;
float currentValue3 = 0.0;
float currentValue4 = 0.0;
int currentLimit = 5;

// servo position
int topArmPos = 180;
int middleArmPos = 45;
int bottomArmPos = 135;
int posGripper =90;
int posRotator = 45;

//===================== setup() ========================


void setup() {
//===== Set Digital Pin Mode =====    
// Set pinmode to read command signal from Receiver.  
  pinMode(CH1,INPUT);   //channel 1
  pinMode(CH2,INPUT);   //channel 2
  pinMode(CH3,INPUT);   //channel 3
  pinMode(CH4,INPUT);   //channel 4
  pinMode(CH5,INPUT);   //channel 5
  pinMode(CH6,INPUT);   //channel 6  
// Set pinmode to read command signal from Test Switch.
  pinMode(buttonPin,INPUT);
// Set pinmode to write command signal to Motor Driver.
  // Motor driver1
  pinMode(INA1,OUTPUT);
  pinMode(INB1,OUTPUT);
  pinMode(PWM1,OUTPUT);
  // Motor driver2
  pinMode(INA2,OUTPUT);
  pinMode(INB2,OUTPUT);
  pinMode(PWM2,OUTPUT);
  // Motor driver3
  pinMode(INA3,OUTPUT);
  pinMode(INB3,OUTPUT);
  pinMode(PWM3,OUTPUT);
  // Motor driver4
  pinMode(INA4,OUTPUT);
  pinMode(INB4,OUTPUT);
  pinMode(PWM4,OUTPUT);
  
// Set pinmode to write command signal to LED.
  pinMode(ledPin1,OUTPUT);
  pinMode(ledPin2,OUTPUT);
  pinMode(ledPin3,OUTPUT);
  pinMode(ledPin4,OUTPUT);
// Assign Servo variable to a servo pin
  myServo1.attach(servo1);
  myServo2.attach(servo2);
  myServo3.attach(servo3);
  myServo4.attach(servo4);
  myGripper.attach(gripper);
  myRotator.attach(rotator);

// Set pinmode to write command signal to Stepper Motor & Driver.
  pinMode(pinDir, OUTPUT);
  pinMode(pinStep, OUTPUT);
  pinMode(pinEnable, OUTPUT);


//===== Initialize Command =====
  // Initialize Motor Driver.
  // Motor driver1
  digitalWrite(INA1,LOW);
  digitalWrite(INB1,LOW);
  analogWrite(PWM1,0);

  // Motor driver2
  digitalWrite(INA2,LOW);
  digitalWrite(INB2,LOW);
  analogWrite(PWM2,0);

  // Motor driver3
  digitalWrite(INA3,LOW);
  digitalWrite(INB3,LOW);
  analogWrite(PWM3,0);

  // Motor driver4
  digitalWrite(INA4,LOW);
  digitalWrite(INB4,LOW); 
  analogWrite(PWM4,0); 

  // Initialize Servo Motor, Set servo to Mid-point.
  myServo1.write(topArmPos);
  myServo2.write(180-topArmPos);
  myServo3.write(bottomArmPos);
  myServo4.write(180 - bottomArmPos);
  myGripper.write(posGripper);
  myRotator.write(posRotator);

  // Initialize Stepper Motor direction, and Step
  digitalWrite(pinEnable, LOW);
     
  // Open Serial port, Set baud rate for serial data transmission.
  Serial.begin(115200); // USB:Rx0,Tx0

  // Returns time(us)
  
  previousLoopTime = micros(); //Returns the number of microseconds since the Arduino board began running the current program. 

} // End SetUp


//======================= loop() ==========================


void loop() {
   
  loopTime = micros()-previousLoopTime;
  
  // if loop time is more than 10000 microseconds, do the next loop.
  // Limit Maximum feedback loop at 100Hz.
  if(loopTime >= 10000) 
  {
    // Set new loop time
    previousLoopTime = micros();

    
    // Read input signal from receiver. PulseIn signal from Receiver vary between 1000 - 10000.
    // Substract 1500 as the offset to set new signal range from (1000, 10000) to (-500, 500) 
    // Also set Deadband limit to the input Signal

    input6 = pulseIn(CH6,HIGH)-1500; //Channel 6
    input6 = Deadband(input6, 100);

    delay(15);
    // Mode
    //Mode 1 Drive Motor input6 < 0
    if (input6 < 0){
      Serial.println("Mode 1");
      input1 = pulseIn(CH1,HIGH)-1500; //Channel 1
      input2 = pulseIn(CH2,HIGH)-1500; //Channel 2
      input3 = pulseIn(CH3,HIGH)-1500; //Channel 3
      input4 = pulseIn(CH4,HIGH)-1500; //Channel 4
      input1 = Deadband(input1,30); //Channel 1
      input2 = Deadband(input2,30); //Channel 2
      input3 = Deadband(input3,30); //Channel 3
      input4 = Deadband(input4,30); //Channel 4
      delay(15);
      // Read Motor's Current From Motor Driver
      // The resolution of Arduino analogRead is 5/1024 Volts/Unit. (10-Bit, Signal vary from 0 to 1023 units)
      // The resolution of Current Sensor from POLOLU VNH5019 is 0.14 Volt/Amp.
      // Convert analogRead signal(Volt) to Current(Amp) by multiply (5/1024)/0.14 = 0.035 Amp/Unit.
      currentValue1 = analogRead(CS1)*0.035; // Motor Driver 1 amp
      currentValue2 = analogRead(CS2)*0.035; // Motor Driver 2 amp
      currentValue3 = analogRead(CS3)*0.035; // Motor Driver 3 amp
      currentValue4 = analogRead(CS4)*0.035; // Motor Driver 4 amp
      delay(15);
      // Robot Direction is controlled by input3 and input4 (CH3, CH4)
      left_w = -input1-input2;
      right_w = -input1+input2;

      // Check Motor Current and Assign Motor Direction
      // Motor can be operated, if motor's current is lower than 5 amp.
      if(currentValue1 < currentLimit) out1 = OutputToMotor1(input1);
        else out1 = OutputToMotor1(0);
      if(currentValue2 < currentLimit) out2 = OutputToMotor1(input2);
        else out2 = OutputToMotor2(0);
      if(currentValue3 < currentLimit) out3 = OutputToMotor3(left_w);
        else out3 = OutputToMotor3(0);    
      if(currentValue4 < currentLimit) out4 = OutputToMotor4(right_w);
        else out4 = OutputToMotor4(0);

      // Drive Motor
      // Assign motor's direction by function OutputToMotor. 
      // Command PWM Duty Cycle to drive motor.
      analogWrite(PWM1,out1);
      analogWrite(PWM2,out2);
      analogWrite(PWM3,out3);
      analogWrite(PWM4,out4);
      
    }
    // Stepper motor mode && Gripper
    if (input6 == 0) {
      // stepper mode
      Serial.println("Mode 2");

      input1 = pulseIn(CH1,HIGH)-1500; //Channel 1
      input2 = pulseIn(CH2,HIGH)-1500; //Channel 2
      input4 = pulseIn(CH4,HIGH)-1500; //Channel 4
      out5 = input4;
      delay(15);
      
      // Turn CW direction Stepper motor
      if (out5 > 200)
      { 
        digitalWrite(pinDir, HIGH);
        for(Index = 0; Index < 2000; Index++)
        {
          if (out5 < 200 && out5 > -200)
          {     
          digitalWrite(pinStep,LOW);
          }
          else
          {
          digitalWrite(pinStep,HIGH);
          delay(5);
          digitalWrite(pinStep,LOW);
          delay(5);
          Serial.println("Stepper CW");
          }
        }

      }
      // Turn CCW direction Stepper motor
      if(out5 < -200)
      {
//        digitalWrite(pinDir, LOW);
//        digitalWrite(pinStep, HIGH);
//        delay(1000);
//        digitalWrite(pinStep, LOW);
//        Serial.println("Stepper CCW");
//        Serial.print("input4");
//        Serial.println( input4);
//        delay(2000);
        digitalWrite(pinDir, LOW);
        for(Index = 0; Index < 2000; Index++)
        {
          if (out5 < 200 && out5 > -200)
          {     
          digitalWrite(pinStep,LOW);
          }
          else
          {
          digitalWrite(pinStep,HIGH);
          delay(5);
          digitalWrite(pinStep,LOW);
          delay(5);
          Serial.println("Stepper CW");
          }
        }
   
      }
      if (out5 < 200 && out5 > -200){
        digitalWrite(pinStep, LOW);
        Serial.println("Stepper Still");
        Serial.print("input4");
        Serial.println( input4);
        delay(5);
      }

      //Gripper
      if (input1 > 200 && posGripper < 180){
        myGripper.write(posGripper);
        posGripper++;
        Serial.println("Gripper +"); 
        delay(15);
      }
      //Gripper
      if (input1 < -200 && posGripper > 0){
        myGripper.write(posGripper);
        posGripper--;
        Serial.println("Gripper -");
        delay(15);
      }
      
      //Rotator
      if (input2 > 200 && posRotator < 180){
        myRotator.write(posRotator);
        posRotator=posRotator+2;
        Serial.println("Rotator +");
        Serial.println(posRotator);
        delay(15);
      }
      //Rotator
      if (input2 < -200 && posRotator > 0){
        myRotator.write(posRotator);
        posRotator=posRotator-2;
        Serial.println("Rotator -");
        Serial.println(posRotator);
        delay(15);
      }
      
    }
    if (input6 > 0) { // Arm servo
      Serial.println("Mode 3");
      input1 = pulseIn(CH1,HIGH)-1500; //Channel 1
      input2 = pulseIn(CH2,HIGH)-1500; //Channel 2
      input4 = pulseIn(CH4,HIGH)-1500; //Channel 4
      delay(15);
      // top servo
      if (input2 > 200 && topArmPos < 180){
        myServo1.write(topArmPos);
        myServo2.write(180-topArmPos);
        topArmPos++;
        Serial.println("Top Arm +");
        digitalWrite(ledPin1, HIGH);
        Serial.println(input2);
        delay(15);
      }
      if (input2 < -200 && topArmPos > 0){
        myServo1.write(topArmPos);
        myServo2.write(180-topArmPos);
        topArmPos--;
        Serial.println("Top Arm -");
        digitalWrite(ledPin1, HIGH);
        Serial.println(input2);
        delay(15);
      }

      // middle servo
//      if (input1 < -200 && middleArmPos < 180){
//        myServo2.write(middleArmPos);
//        middleArmPos++;
//        Serial.println("Middle Arm +");
//        digitalWrite(ledPin2, HIGH);
//        delay(15);
//      }
//      if (input1 > 200 && middleArmPos > 0){
//        myServo2.write(middleArmPos);
//        middleArmPos--;
//        Serial.println("Middle Arm -");
//        digitalWrite(ledPin2, HIGH);
//        delay(15);
//      }

      // bottom servos
      if (input1 > 200 && bottomArmPos < 180){
        myServo3.write(bottomArmPos);
        myServo4.write(180-bottomArmPos);
        bottomArmPos++;
        Serial.println("Bottom Arm +");
        digitalWrite(ledPin3, HIGH);
        digitalWrite(ledPin4, HIGH);
        delay(15);
      }
      if (input1 < -200 && bottomArmPos > 0){
        myServo3.write(bottomArmPos);
        myServo4.write(180-bottomArmPos);
        bottomArmPos--;
        Serial.println("Bottom Arm -");
        digitalWrite(ledPin3, HIGH);
        digitalWrite(ledPin4, HIGH);
        delay(15);
      }
      else {
        digitalWrite(ledPin1, LOW);
        digitalWrite(ledPin2, LOW);
        digitalWrite(ledPin3, LOW);
        digitalWrite(ledPin4, LOW);
        delay(15);
      }
      
    }
  

    // Print
//    Serial.print("M 1 = ");
//    Serial.print(input1);
//    Serial.print("\t M 2 = ");
//    Serial.print(input2);
//    Serial.print("\t M 3 = ");
//    Serial.print(input3);
//    Serial.print("\t M 4 = ");
//    Serial.print(input4);
//    Serial.print("\t M 5 = ");
//    Serial.print(input5);
    Serial.print("\t M 6= ");
    Serial.print(input6);
//    Serial.print("\t LoopTime = ");
//    Serial.println(loopTime);
  
  } // End if
} // End loop


//=============== Function Declaration ===============


//===== double Deadband(double value,double limit) =====
  //===== Set Dead Band =====
  // If the input signal from receiver is in the band limit, set input signal to 0.0.
  double Deadband(double value,double limit)
  {
    double temp = 0.0;
    if(value >= limit) temp = value-limit;
    else if(value <= -limit) temp = value+limit;
    else temp = 0.0;
    return temp;
  }


//===== int OutputToMotor(int value) ======
  //===== Assign Motor's Direction and Scale Down Input Signal =====
  // value must be positive and scaled down to fit 8-Bit PWM Range. 

  // Motor 1
  int OutputToMotor1(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      // CW
      digitalWrite(INA1,LOW);
      digitalWrite(INB1,HIGH);
//      digitalWrite(ledPin1, LOW);
      temp = map(value,0,500,0,255);
    }else{
      // CCW
      digitalWrite(INA1,HIGH);
      digitalWrite(INB1,LOW);
//      digitalWrite(ledPin1,HIGH);
      temp = map(-value,0,500,0,255);
    }
    return temp;
  }
  
  // Motor 2
  int OutputToMotor2(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      digitalWrite(INA2,LOW);
      digitalWrite(INB2,HIGH);
//      digitalWrite(ledPin2, LOW);
      temp = map(value,0,500,0,255);
    }else{
      digitalWrite(INA2,HIGH);
      digitalWrite(INB2,LOW);
//      digitalWrite(ledPin2, HIGH);
      temp = map(-value,0,500,0,255);
    }
    return temp;
  }
  
  // Motor 3
  int OutputToMotor3(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      digitalWrite(INA3,LOW);
      digitalWrite(INB3,HIGH);
//      digitalWrite(ledPin3, LOW);
      temp = map(value,0,500,0,255);
    }else{
      digitalWrite(INA3,HIGH);
      digitalWrite(INB3,LOW);
//      digitalWrite(ledPin3, HIGH);
      temp = map(-value,0,500,0,255);
    }
    return temp;
  }
  
  // Motor 4
  int OutputToMotor4(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      digitalWrite(INA4,LOW);
      digitalWrite(INB4,HIGH);
      temp = map(value,0,500,0,255);
    }else{
      digitalWrite(INA4,HIGH);
      digitalWrite(INB4,LOW);
      temp = map(-value,0,500,0,255);
    }
    return temp;
  }
