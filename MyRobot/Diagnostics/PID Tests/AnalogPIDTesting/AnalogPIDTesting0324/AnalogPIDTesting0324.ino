#include <NewPing.h>

/*------------------------------------------------------------------

-------------------------------------------------------------------*/

//#include "robotDefines.h"

String command;
String device;

// BT Module
#include <SoftwareSerial.h>
SoftwareSerial BT1(2, 3);  // pin 2 is Rx and pin 3 is Tx

/////////////////////////////////////////////////////////////////////

int mode = 0;
int saveError  = 0;
int numSensors = 0;

int trippedLeft  = 0;
int trippedRight = 0;

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2

// configure ultrasonic rangefinger pins...
#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_REV 11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN_FWD 13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

// sensor settings...
float a1Val;
float a2Val;
float a3Val;
float a4Val;
float a5Val;

float a1Min = 21;
float a2Min = 22;
float a3Min = 20;
float a4Min = 12;
float a5Min = 18;

float a1Max = 350;
float a2Max = 350;
float a3Max = 350;
float a4Max = 350;
float a5Max = 350;

float a1Norm = 0;
float a2Norm = 0;
float a3Norm = 0;
float a4Norm = 0;
float a5Norm = 0;

int a1Dig = 0;
int a2Dig = 0;
int a3Dig = 0;
int a4Dig = 0;
int a5Dig = 0;

float rPosition = 0;
float nPosition = 0;
float ePosition = 0;  // my own position algorithm

// configure motor controller pins... 
// right motors:
int enR = 10;
int in1 = 9;
int in2 = 8;
// left motors:
int enL = 5;
int in3 = 7;
int in4 = 6;

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 4;         // right switches pin (digital)
//const int buttonTwo = 0;       // left  switches pin (analog)

// LFSensor more to the Left is "0"
const int lineFollowSensor0 = 1; 
const int lineFollowSensor1 = 2; 
const int lineFollowSensor2 = 3; 
const int lineFollowSensor3 = 4;
const int lineFollowSensor4 = 5;

int LFSensor[5]={0, 0, 0, 0, 0};


float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

#define RIGHT 1
#define LEFT -1

int power = 100;  //170;          // fwd and rev: ok = 200;
float adjRTurn = 30;
float adjLTurn = 33;

//--------------------------------------------------------
// Make changes here:

//int    errorCount    =  10; // 25;
//float  errorCorr     =  1.1;
//float  orgKd = 40;

///////////////////////////////////////////////////////////////////////////////////


int    iniMotorPower = 45; //50;      
int    adjR =  0;
int    adjL =  40;

float    Ki   =  0.00; // zero  ok        

int debug = 0;  // display serial i/o data

 
float    Kp   =  0.15;  // 1.5;  ok   
float    Kd   =  0.875; //.65;  // 9.0;  ok

///////////////////////////////////////////////////////////////////////////////////


//------------------------------------------------------------------------
float normalize(float nx, float nMin, float nMax) 
{
  float ny = ((nx - nMin) * 1000) / (nMax - nMin);
  
  if (false) {  
    Serial.print("Normalized -> ");
    Serial.print(ny);
    Serial.print(" = (");
    Serial.print(nx);
    Serial.print(" - ");
    Serial.print(nMin);
    Serial.print(") * 1000 / (");
    Serial.print(nMax);
    Serial.print(" - ");
    Serial.print(nMin);
    Serial.println(")");
  }
  
  return ny;
}


//--------------------------------------------------------
void calculatePID()
{
  P = error;
  I = I + error;
  D = abs(error - previousError);  // ???
  if (trippedRight) { D = D * -1; }
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
  
  
  if (debug) {
    Serial.println ("");
    Serial.print (" P * Kp:   ");
    Serial.print (P * Kp);
    Serial.print (" = ");
    Serial.print (P);
    Serial.print (" * ");
    Serial.println (Kp);
 
    Serial.print (" I * Ki:   ");
    Serial.print (I * Ki);
    Serial.print (" = ");
    Serial.print (I);
    Serial.print (" * ");
    Serial.println (Ki);
 
    Serial.print (" D * Kd:   ");
    Serial.print (D * Kd);
    Serial.print (" = ");
    Serial.print (D);
    Serial.print (" * ");
    Serial.println (Kd);
 
    Serial.print (" PID: ");
    Serial.println (PIDvalue);
  }
  
}

//---------------------------------------------------
void motorPIDcontrol()
{
  
  int rightMotorSpeed = iniMotorPower + adjR;
  int leftMotorSpeed  = iniMotorPower + adjL;
  
  if (debug) {
    Serial.println("");
    Serial.println("motorPIDcontrol...");
    
    Serial.print("  rightMotorSpeed: ");
    Serial.print(rightMotorSpeed);
    Serial.print(" = ");
    Serial.print(iniMotorPower);
    Serial.print(" + ");
    Serial.println(adjR);
    
    Serial.print("  leftMotorSpeed:  ");
    Serial.print(leftMotorSpeed);
    Serial.print(" = ");
    Serial.print(iniMotorPower);
    Serial.print(" + ");
    Serial.println(adjL);
  }
  
    Serial.print("P: ");
    Serial.print(P);
    Serial.print("; D = ");
    Serial.println(D);

  if (trippedLeft)  // tripping left sensors, turn left...
  { 
    Serial.print("  Tripped a LEFT  sensor, turning LEFT by  PID = ");
    Serial.println(PIDvalue);
    leftMotorSpeed  = leftMotorSpeed  - PIDvalue; 
    rightMotorSpeed = rightMotorSpeed + PIDvalue;
  
  }
  
  if (trippedRight)  // tripping right sensors, turn right...
  { 
    rightMotorSpeed = rightMotorSpeed + PIDvalue;
    leftMotorSpeed  = leftMotorSpeed  - PIDvalue; 
   
  }
  
  // The motor speeds should not exceed the function limits  
  if (rightMotorSpeed < -255)   { 
      Serial.print("* RIGHT MotorSpeed too  LOW: ");
      Serial.println(rightMotorSpeed);  
      rightMotorSpeed = -255;   
      //leftMotorSpeed  = leftMotorSpeed - abs(255 + rightMotorSpeed); 
      //if (leftMotorSpeed  = leftMotorSpeed   - rightMotorSpeed; 
  }
  if (leftMotorSpeed < -255)    {
      Serial.print("! LEFT  MotorSpeed too  LOW: ");
      Serial.println(leftMotorSpeed);  
      //rightMotorSpeed  = rightMotorSpeed - abs(255 + leftMotorSpeed); 
      leftMotorSpeed  = -255;   
  }
       
  if (rightMotorSpeed > 255)  { 
      Serial.print("! RIGHT MotorSpeed too HIGH: ");
      Serial.println(rightMotorSpeed);  
      //leftMotorSpeed  = leftMotorSpeed - (rightMotorSpeed - 255); 
      rightMotorSpeed = 255; 
  }
  if (leftMotorSpeed > 255)  {
      Serial.print("* LEFT  MotorSpeed too HIGH: ");
      Serial.println(leftMotorSpeed);  
      //rightMotorSpeed  = rightMotorSpeed - (leftMotorSpeed - 255); 
      leftMotorSpeed  = 255; 
  }
  
  Serial.println("");

  //iniMotorPower + adjR
  if (rightMotorSpeed > 0) {
    Serial.print("  RIGHT motor FORWARD: ");   
    Serial.println(rightMotorSpeed);  
     // right motors forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    Serial.print("  RIGHT motor REVERSE: ");  
    Serial.println(rightMotorSpeed);  
    // right motors forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);    
  }
  if (leftMotorSpeed > 0) {
    Serial.print("  LEFT  motor FORWARD: ");  
    Serial.println(leftMotorSpeed);  
     // left motors forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    Serial.print("  LEFT  motor REVERSE: ");  
    Serial.println(leftMotorSpeed);  
    // left motors forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);    
  }
  
  //deadzone adjust: -40 to 40 won't turn the motors so adjust for this gap
  //if (rightMotorSpeed > 0 and rightMotorSpeed < 40) {
    //if (PID > 0) {
  //  rightMotorSpeed = rightMotorSpeed + 40;
    //} else {
    //rightMotorSpeed = rightMotorSpeed - 80;
    //}
  //}
  //if (rightMotorSpeed < 0 and rightMotorSpeed > -40) {
  //  rightMotorSpeed = rightMotorSpeed - 40;
  //}
  //if (leftMotorSpeed > 0 and leftMotorSpeed < 40) {
  //  leftMotorSpeed = leftMotorSpeed + 40;
  //}
  //if (leftMotorSpeed < 0 and leftMotorSpeed > -40) {
  //  leftMotorSpeed = leftMotorSpeed - 40;
  //}  
  
  // set motors to speed in possible range 0~255
  analogWrite(enR, abs(rightMotorSpeed));
  analogWrite(enL, abs(leftMotorSpeed));
  
  if (numSensors == 3)  //very sharp angle; severe turn
  {
    //delay(1000);
    Serial.println ("numSensors = 3"); 
  }
}

//--------------------------------------------------------
void checkPIDvalues()
{
  
  BT1.print("PID: ");
  BT1.print(Kp);
  BT1.print(" - ");
  BT1.print(Ki);
  BT1.print(" - ");
  BT1.println(Kd); 
    
  if (debug) {
    Serial.print("PID: ");
    Serial.print(Kp);
    Serial.print(" - ");
    Serial.print(Ki);
    Serial.print(" - ");
    Serial.println(Kd);  
  }
}

//-----------------------------------------------------------------------------
 void checkBTcmd()  
 { 
   
   while (BT1.available())   //Check if there is an available byte to read
   {
     delay(10);              //Delay added to make thing stable 
     char c = BT1.read();    //Conduct a serial read
     device += c;            //build the string.
   }  
   if (device.length() > 0) 
   {
     if (debug) { 
       Serial.print("Command received from BT ==> ");
       Serial.println(device); 
     }
     command = device;
     device ="";             //Reset the variable
     BT1.flush();
   } 
}

//------------------------------------------------------------------------
void manualCmd()
{
  switch (command[0])
  {
    
    case 's': 
      motorStop(); //turn off both motors
      break;
      
    case 'g':
      mode = FOLLOWING_LINE;
        if (debug) { Serial.println("Checking the PID values"); }
      checkPIDvalues();

      break;

    case 'f':  
      motorForward();  
      break;

    case 'r':     
      motorTurn(RIGHT, 30);
      motorStop();      
      break;

   case 'l': 
      motorTurn(LEFT, 30);
      motorStop();
      break;
    
    case 'b':  
      motorBackward();
      break;
      
    case 'p':
      Kp = command[2];
      break;
    
    case 'i':
      Ki = command[2];
      break; 
    
    case 'd':
      Kd = command[2];
      break;
  }
}

//------------------------------------------------------------------------
void sendBTdata (int data) // send data to BT

{
    //digitalWrite (ledPin, HIGH);
    BT1.print("Data from Arduino");
    BT1.print(data);
    BT1.print(" xx");
    BT1.println('\n');
}

//------------------------------------------------------------------------
unsigned int digitize(int thisVal)
{
  if (thisVal < 360)   { return 1; }
  else                 { return 0; }
}    

//--------------------------------------------- 
void motorStop()
{ 
  // right motors stop
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  // left  motors stop
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  delay(200);
}

//--------------------------------------------- 
void motorForward()
{
  
  Serial.println ("motorForward");
  
  // right motors forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // left motors forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
    
  // set motors to speed 
  analogWrite(enR, power + adjR);
  analogWrite(enL, power + adjL);
 
}

//---------------------------------------------
void motorBackward()
{  
  
  Serial.println ("motorBackward");
  
  // right motors backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // left motors backward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // set motors to speed 
  analogWrite(enR, power);
  analogWrite(enL, power);

}

//---------------------------------------------
void motorFwTime (unsigned int time)
{
  
  Serial.println ("motorFwTime");

  motorForward();
  delay (time);
  motorStop();
  
}

//---------------------------------------------
void motorBwTime (unsigned int time)
{
  
  Serial.println ("motorBwTime");

  motorBackward();
  delay (time);
  motorStop();
  
}

//------------------------------------------------
void motorTurn(int direction, int degrees)
{
  
  Serial.println ("motorTurn");
  
  int adjTurn = 0;
  
  if (direction == RIGHT) {
    
    // right motors backward
    //digitalWrite(in1, LOW);
    //digitalWrite(in2, HIGH);

    // left motors forward
    //digitalWrite(in3, HIGH);
    //digitalWrite(in4, LOW);
    
    adjTurn = adjRTurn;
  }
  
  if (direction == LEFT) {
    
    // right motors forward
    //digitalWrite(in1, HIGH);
    //digitalWrite(in2, LOW);
    
    // left motors backward
    //digitalWrite(in3, LOW);
    //digitalWrite(in4, HIGH);

    adjTurn =adjLTurn;
  }
  
  // set motors to speed
  analogWrite(enR, iniMotorPower+ 100);
  analogWrite(enL, iniMotorPower);

  delay (round(adjTurn * degrees + 1));
  motorStop();
}


//-------------------------------------------------------------
// read line sensors  

void readAnalogSensors()
{
  mode = FOLLOWING_LINE; 
  error = previousError;  // cover the patterns not originally anticipated

  a1Val = analogRead(A1);
  a2Val = analogRead(A2);
  a3Val = analogRead(A3);
  a4Val = analogRead(A4);
  a5Val = analogRead(A5);
  
  // implement min values recorded during training...
  if (a1Val < a1Min) { a1Val = a1Min; }
  if (a2Val < a2Min) { a2Val = a2Min; }
  if (a3Val < a3Min) { a3Val = a3Min; }
  if (a4Val < a4Min) { a4Val = a4Min; }
  if (a5Val < a5Min) { a5Val = a5Min; }
  
  if (false) {
    if (a1Val > a1Max) { a1Val = a1Max; }
    if (a2Val > a2Max) { a2Val = a2Max; }
    if (a3Val > a3Max) { a3Val = a3Max; }
    if (a4Val > a4Max) { a4Val = a4Max; }
    if (a5Val > a5Max) { a5Val = a5Max; } 
  }
  
  if (debug) {
    Serial.print("Raw (limited): ");
    Serial.print(a1Val);
    Serial.print(" : ");
    Serial.print(a2Val);
    Serial.print(" : ");
    Serial.print(a3Val);
    Serial.print(" : ");
    Serial.print(a4Val);
    Serial.print(" : ");
    Serial.println(a5Val);
  }

  a1Dig = digitize(a1Val);
  a2Dig = digitize(a2Val);
  a3Dig = digitize(a3Val);
  a4Dig = digitize(a4Val);
  a5Dig = digitize(a5Val);
  
  numSensors = a1Dig + a2Dig + a3Dig + a4Dig + a5Dig; 
  
  if (true) {
    Serial.println(""); 
    Serial.print("Pattern:  ");
    Serial.print(a1Dig);
    Serial.print(" : ");
    Serial.print(a2Dig);
    Serial.print(" : ");
    Serial.print(a3Dig);
    Serial.print(" : ");
    Serial.print(a4Dig);
    Serial.print(" : ");
    Serial.print(a5Dig);
    Serial.println(""); 
  }
     
  int pDig = a5Dig + a4Dig * 10 + a3Dig * 100 + a2Dig * 1000 + a1Dig * 10000;  // set pattern to number
    
  a1Norm = normalize(a1Val, a1Min, a1Max);
  a2Norm = normalize(a2Val, a2Min, a2Max);
  a3Norm = normalize(a3Val, a3Min, a3Max);
  a4Norm = normalize(a4Val, a4Min, a4Max);
  a5Norm = normalize(a5Val, a5Min, a5Max);
                                                                              //    SENSOR PATTERNS
                                                                              //    =============== 
  if (numSensors == 1 or numSensors ==2) {                                    //    L  1-2-3-4-5  R 
    if (pDig == 10000) { a3Val = a3Max; a4Val = a4Max; a5Val = a5Max; }       //       * - - - -      
    if (pDig == 11000) { a4Val = a4Max; a5Val = a5Max; }                      //       * * - - -
    if (pDig == 1000)  { a4Val = a4Max; a5Val = a5Max; }                      //       - * - - -
    if (pDig == 1100)  { a5Val = a5Max; }                                     //       - * * - -
    if (pDig == 100)   { a1Val = a1Max; a5Val = a5Max; }                      //       - - * - -    center
    if (pDig == 110)   { a1Val = a1Max; }                                     //       - - * * - 
    if (pDig == 10)    { a1Val = a1Max; a2Val = a2Max; }                      //       - - - * -
    if (pDig == 11)    { a1Val = a1Max; a2Val = a2Max; }                      //       - - - * *
    if (pDig == 1)     { a1Val = a1Max; a2Val = a2Max; a3Val = a3Max; }       //       - - - - *
  }  
  if (numSensors == 0) {error = 0; mode = NO_LINE;}                           //       - - - - -    reverse
  if (numSensors == 5) {error = 0; mode = STOPPED;}                           //       * * * * *    stop
  
  int pos = 0;
  if (numSensors == 1 or numSensors ==2) {
    if (pDig == 10000) { pos = a1Norm; }
    if (pDig == 11000) { pos = a1Norm + a2Norm; }
    if (pDig == 1000)  { pos = 1000 + a2Norm; }
    if (pDig == 1100)  { pos = 1000 + a2Norm + a3Norm; }
    if (pDig == 100)   { pos = 2000 + a3Norm; }
    if (pDig == 110)   { pos = 2000 + a3Norm + a4Norm; }
    if (pDig == 10)    { pos = 3000 + a4Norm; }
    if (pDig == 11)    { pos = 3000 + a4Norm + a5Norm; }
    if (pDig == 1)     { pos = 4000 + a5Norm; } 
    
    rPosition = pos;
    //previousError = error;
    error = (2000 - rPosition) / 10;     
    if (pDig == 100)   { error = 0; }    
  }  

  if (pDig > 111) { trippedLeft  = 1; trippedRight = 0; }
  else            { trippedRight = 1; trippedLeft  = 0; }
  
  if (debug) {
    
    Serial.print("pDig = ");
    Serial.println(pDig);
    Serial.println("");


    Serial.print("a1Norm = ");
    Serial.print(a1Norm);
    Serial.print(" : ");
    Serial.print(a1Min);
    Serial.print(" to ");
    Serial.println(a1Max);
    
    Serial.print("a2Norm = ");
    Serial.print(a2Norm);
    Serial.print(" : ");
    Serial.print(a2Min);
    Serial.print(" to ");
    Serial.println(a2Max);
    
    Serial.print("a3Norm = ");
    Serial.print(a3Norm);
    Serial.print(" : ");
    Serial.print(a3Min);
    Serial.print(" to ");
    Serial.println(a3Max);
    
    Serial.print("a4Norm = ");
    Serial.print(a4Norm);-
    Serial.print(" : ");
    Serial.print(a4Min);
    Serial.print(" to ");
    Serial.println(a4Max);
    
    Serial.print("a5Norm = ");
    Serial.print(a5Norm);
    Serial.print(" : ");
    Serial.print(a5Min);
    Serial.print(" to ");
    Serial.println(a5Max);

    Serial.println("");  
  }
   
  if (true) {
    Serial.print("Position: ");
    Serial.print(rPosition);
    Serial.print(";  Error: ");
    Serial.println(error);
    Serial.println("");  
  }
}


//======================================================================== setup
void setup()
{
  Serial.begin(9600);
  BT1.begin(9600);
   
  // set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // initialize the pushbuttons:
  pinMode(buttonPin, INPUT);
  
  Serial.println("TRSensor example");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  
  
  BT1.print("Check the PID constants to be sent to Robot");
  BT1.println('\n');
 
 
  // wait until button is pushed again - signalling training start
  while (!digitalRead(buttonPin))
  {      
    // set sensor limits
    a1Val = analogRead(A1);
    a2Val = analogRead(A2);
    a3Val = analogRead(A3);
    a4Val = analogRead(A4);
    a5Val = analogRead(A5);
    
    if (a1Val < a1Min) { a1Min = a1Val; }
    if (a1Val > a1Max) { a1Max = a1Val; }    
    if (a2Val < a2Min) { a2Min = a2Val; }
    if (a2Val > a2Max) { a2Max = a2Val; }
    if (a3Val < a3Min) { a3Min = a3Val; }
    if (a3Val > a3Max) { a3Max = a3Val; }    
    if (a4Val < a4Min) { a4Min = a4Val; }
    if (a4Val > a4Max) { a4Max = a4Val; }    
    if (a5Val < a5Min) { a5Min = a5Val; }
    if (a5Val > a5Max) { a5Max = a5Val; }        
  }

  if (debug) {
    Serial.println("Sensor MIN - MAX");
    Serial.print("A1 -> ");
    Serial.print(a1Min);
    Serial.print(" : ");
    Serial.println(a1Max);
    Serial.print("A2 -> ");
    Serial.print(a2Min);
    Serial.print(" : ");
    Serial.println(a2Max);
    Serial.print("A3 -> ");
    Serial.print(a3Min);
    Serial.print(" : ");
    Serial.println(a3Max);
    Serial.print("A4 -> ");
    Serial.print(a4Min);
    Serial.print(" : ");
    Serial.println(a4Max);
    Serial.print("A5 -> ");
    Serial.print(a5Min);
    Serial.print(" : ");
    Serial.println(a5Max);
  }
  
  delay(50);
   
  while (!digitalRead(buttonPin) && !mode)
  {  
    checkBTcmd();  // verify if a comand is received from BT remote control
    manualCmd ();    
    command = "";  
  }
  checkPIDvalues();
 
} //end newSetup



//======================================================================== test
void xloop()
{
  // right motors move FWD at power setting of:  45
  // left  motors move FWD at power setting of:  75
     //                              iniMotorPower:  45;
     //                                      adjLF:  30; 
     
  // right motors move REV at power setting of: -45
  // left  motors move REV at power setting of: -80
     //                              iniMotorPower: -70;
     //                                      adjLR: -20;
     
    iniMotorPower =  45;    
    adjL  =          30;    // FWD adjust value
  
  //iniMotorPower = -70;    
  //adjLR =         -20;    // REV adjust value


  // testing: try 50 and 80 fwd and rev, with ladj of 30:
    iniMotorPower =  60;   //ok
             adjL =  40;   //ok 
  //iniMotorPower = -70;   //ok
  //         adjL = -15;   //ok

  int rightMotorSpeed = iniMotorPower;
  int leftMotorSpeed  = iniMotorPower + adjL;

  Serial.println("Determining DEAD ZONE Limits");
  Serial.println("");
  
  if (rightMotorSpeed > 0) {
    Serial.print("  RIGHT motor FORWARD: ");   
    Serial.println(rightMotorSpeed);  
     // right motors forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    Serial.print("  RIGHT motor REVERSE: ");  
    Serial.println(rightMotorSpeed);  
    // right motors forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);    
  }
  if (leftMotorSpeed > 0) {
    Serial.print("  LEFT  motor FORWARD: ");  
    Serial.println(leftMotorSpeed);  
     // left motors forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    Serial.print("  LEFT  motor REVERSE: ");  
    Serial.println(leftMotorSpeed);  
    // left motors forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);    
  }
    
  // set motors to speed in possible range 0~255
  analogWrite(enR, abs(rightMotorSpeed));
  analogWrite(enL, abs(leftMotorSpeed));
  
  

}

//======================================================================== loop
void loop() 
{
 
  if (debug) {
    Serial.println("");
    Serial.println("---------------------------------------------------");
  }
  
  readAnalogSensors();
  
  switch (mode)
  {
    case STOPPED: 
      motorStop();
       while (!digitalRead(buttonPin) && !mode)
      {  
        checkBTcmd();
        manualCmd ();
        command = "";
      }
      break;

    case NO_LINE:  
      motorStop();
      //if (false) {//(saveError < errorCount) {
      //  saveError = saveError + 1;
      //  error = previousError * errorCorr;
      //  calculatePID();
      //  motorPIDcontrol();
      //}
      break;

    case FOLLOWING_LINE:   
      calculatePID();
      //checkPIDvalues();
      motorPIDcontrol();   
 
      break;     
  }
  
  //delay(50);
  
}


