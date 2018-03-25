#include <NewPing.h>

/*------------------------------------------------------------------
Smart Robot - Line Follower with programable PID controller via BT
==> Basic movement based on Nano Mouse Robot, developed by Michael Backus (http://www.akrobotnerd.com/ )
==> Line follow based on http://samvrit.tk/tutorials/pid-control-arduino-line-follower-robot/?ckattempt=1

Marcelo Jose Rovai - 06 April, 2016 - Visit: http://mjrobot.org

Converted from servo controls to motor controls 
-------------------------------------------------------------------*/

//#include "robotDefines.h"

String command;
String device;

// BT Module
#include <SoftwareSerial.h>
SoftwareSerial BT1(2, 3);  // pin 2 is Rx and pin 3 is Tx

/////////////////////////////////////////////////////////////////////

int mode = 0;
int saveError = 0;
int numSensors= 0;

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2

// configure ultrasonic rangefinger pins...
#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_REV 11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN_FWD 13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


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

//const int adjR = 0;

//int adjL = 45;  //ok for 70;  39; ok for 55//ok for power = 50 //54 for power = 100

//okint    iniMotorPower =  70; //55; //50; too low, speed not consistant
//int    iniMotorPower =  50; //55; //50; too low, speed not consistant
//int    iniMotorPower =  45; 
int    errorCount    =  10; // 25;
float  errorCorr     = 1.1;
float orgKd = 40;
int adjR =  0; //38; //45;  //ok for 70;  39; ok for 55//ok for power = 50 //54 for power = 100

// SUNDAY: collect data on right turn working; left turn NOT working (maybe tape sucks?)

///////////////////////////////////////////////////////////////////////////////////
// adjl 42/45;  power 40; Kp 3; Kd 44;  Ki 1.5;
//       45;          40;    8;    39;     0.5
//                    40     9     37
// no                        9     36
//
//  right ok          45     5     38      0.0
// 
// 45  45   5   40  .5 almost did a left turn... not so good on right tho...

// 45  45   4   40  .5 dd ok on right
// 45  45   5   41  .0 dd ok on right

// 47  45   6   44   0  did ok on right - good speed too; left still sux

// 48  47   8   46   0  did a sloppy right turn

// 48  38   8   46   0  did an ok right turn
// 54  40   5.5 48   0

// 60   0   7   48   0  left turn! once.

// 60   0   3.5 50   0  left turn w/ better tracking
// 60   0   3.5 50   0  left turn w/ better tracking



int iniMotorPower = 47;     //40; //43; //; //43; //40;  //45; //55; //80; 

int    adjL   =  45;  //38; //45;  //ok for 70;  39; ok for 55//ok for power = 50 //54 for power = 100
 

float    Kp   =  6;   //6;   //11; //3.5; //4.0;   //1.8;   //2.0; //2.4; ~//2.2; ok//2; //.75; lo//2.5; //1.0; //2; // 1.3;  //1.60;//1.4; //2.2; //1.6; //1.5; //2.0; //1.70; 
                      //1.25;  //1.1; //1.0; too low//1.25; liking it//1.5; ok//3.50;  //4.0; //2.6; //2;x //2.5; //3.0;x//3.25; //3.0; 
                      //2.5; no//4.0; //4.25; //4.5; hi//5; hi//6 too high , see 8; //3.75; sm as 4 
                      //3.5; goes out starting on os//4; works but uses 5 sensors starting on os
                      // 8; overcorrects - too hi // 3.5; //2.7;  //2.0;  //1.0;too low, works using all 5 sensors
                      //?3.0; //2.5;  // use this //??? 2.6; // 2.7; //2.5; better than 2.2! //2.2; will do//5; too high//3.50; barely pass 
                      // 2.8; too high?  //1.75; too low?    //2.00; maybe higher?//3.0; too high//1.0; too low//1.5; kinda ok
                      // 2.5; not bad    //4; //3.5; ok by itself but barely //1.8; //.75;
float    Kd   =  34.5;  //38; sucks//42; sucks//39;sucked //38; //45; //35; //40;//41; neither great //~42; ok r, //43; tracing R ok slow//44;   //40;    //60; // 80;  //50;  //47; 
                      //45; still needs to do more!? (how's batt?)
                      //37; a little better than 35     
                      //35; slow enuf to see turn, not doing enuf to make the turn??? 
                      //30; almost worked once out of three to the right, didn't respond left well
                      //25; straight great but does too little on turn
                      //40; jerky but follows straight line  
                      //50; totally f'd up 

float    Ki    =  0;  //.75; //.5;       //.25; //.5; //1.5;  //1.5;  //this parameter sucks dick! //2;  //10; //0;            //.2;  


//--------------------------------------------------------
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}

//---------------------------------------------------
void motorPIDcontrol()
{
  
  Serial.println ("");
  Serial.println ("motorPIDcontrol");
  
  // right motors forward
  //digitalWrite(in1, HIGH);
  //digitalWrite(in2, LOW);
  
  // left motors forward
  //digitalWrite(in3, HIGH);
  //digitalWrite(in4, LOW);
  
  int rightMotorSpeed = iniMotorPower + adjR;
  int leftMotorSpeed  = iniMotorPower + adjL;
  
  
  if (PIDvalue < 0)  // tripping right sensors, turn left...
  { 
    Serial.print("tripped right sensor(s): adjusting left by PID: ");
    Serial.println(PIDvalue);
    leftMotorSpeed  = leftMotorSpeed  - PIDvalue; 
    rightMotorSpeed = rightMotorSpeed + PIDvalue;
  
  }
  
  if (PIDvalue > 0)  // tripping left sensors, turn right...
  { 
    Serial.print("tripped left sensor(s): adjusting right by PID: ");
    Serial.println(PIDvalue);
    rightMotorSpeed = rightMotorSpeed + PIDvalue;
    leftMotorSpeed  = leftMotorSpeed  - PIDvalue; 
   
  }
  
  // The motor speed should not exceed the max PWM value  
  if (rightMotorSpeed < -255)   { 
      Serial.print("*** rightMotorSpeed too low: ");
      Serial.println(rightMotorSpeed);  
      leftMotorSpeed  = leftMotorSpeed - abs(255 + rightMotorSpeed); 
      //leftMotorSpeed  = leftMotorSpeed   - rightMotorSpeed; 
      rightMotorSpeed = -255;   
  }
  if (leftMotorSpeed < -255)    {
      Serial.print("*** leftMotorSpeed  too low: ");
      Serial.println(leftMotorSpeed);  
      rightMotorSpeed  = rightMotorSpeed - abs(255 + leftMotorSpeed); 
      leftMotorSpeed  = -255;   
  }
       
  if (rightMotorSpeed > 255)  { 
      Serial.print("*** rightMotorSpeed too high: ");
      Serial.println(rightMotorSpeed);  
      leftMotorSpeed  = leftMotorSpeed - (rightMotorSpeed - 255); 
      rightMotorSpeed = 255; 
  }
  if (leftMotorSpeed > 255)  {
     Serial.print("*** leftMotorSpeed  too high: ");
     Serial.println(leftMotorSpeed);  
     rightMotorSpeed  = rightMotorSpeed - (leftMotorSpeed - 255); 
     leftMotorSpeed  = 255; 
  }
  

  if (rightMotorSpeed > 0) {
    Serial.print("Right motor FORWARD: ");  
    Serial.println(abs(rightMotorSpeed));  
     // right motors forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    Serial.print("Right motor REVERSE: ");  
    Serial.println(abs(rightMotorSpeed));  
    // right motors forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);    
  }
  if (leftMotorSpeed > 0) {
    Serial.print("Left  motor FORWARD: ");  
    Serial.println(abs(leftMotorSpeed));  
     // left motors forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    Serial.print("Left  motor REVERSE: ");  
    Serial.println(abs(leftMotorSpeed));  
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

  //Serial.print   (PIDvalue);
  //Serial.print   (" ==> Left, Right:  ");
  //Serial.print   (leftMotorSpeed);
  //Serial.print   ("   ");
  //Serial.println (rightMotorSpeed);
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
    
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);  
  
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
     Serial.print("Command received from BT ==> ");
     Serial.println(device); 
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
      Serial.println("Checking the PID values");
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


unsigned int digitize(int thisVal)
{
  if (thisVal > 255) {
    return 1;
    }
  else {
    return 0;
  }
}    

//-----------------------------------------------
void testLineFollowSensors()
{
     //int LFS0 = analogRead(lineFollowSensor0);
     //int LFS1 = analogRead(lineFollowSensor1);
     //int LFS2 = analogRead(lineFollowSensor2);
     //int LFS3 = analogRead(lineFollowSensor3);
     //int LFS4 = analogRead(lineFollowSensor4);
     
     int LFS0 = (analogRead(lineFollowSensor0));
     int LFS1 = (analogRead(lineFollowSensor1));
     int LFS2 = (analogRead(lineFollowSensor2));
     int LFS3 = (analogRead(lineFollowSensor3));
     int LFS4 = (analogRead(lineFollowSensor4));

     Serial.print ("LFS: L  0 1 2 3 4  R ==> "); 
     Serial.print (LFS0); 
     Serial.print (" | ");
     Serial.print (LFS1); 
     Serial.print (" | ");
     Serial.print (LFS2); 
     Serial.print (" | ");
     Serial.print (LFS3); 
     Serial.print (" | ");
     Serial.print (LFS4); 
     Serial.print ("  ==> ");
    
     Serial.print (" P: ");
     Serial.print (P);
     Serial.print (" I: ");
     Serial.print (I);
     Serial.print (" D: ");
     Serial.print (D);
     Serial.print (" PID: ");
     Serial.println (PIDvalue);
}


void motorStop()
{
  
  // right motors stop
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  // left motors stop
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
/* read line sensors values 

Sensor Array 	Error Value
0 0 0 0 1	 4              
0 0 0 1 1	 3              
0 0 0 1 0	 2              
0 0 1 1 0	 1              
0 0 1 0 0	 0              
0 1 1 0 0	-1              
0 1 0 0 0	-2              
1 1 0 0 0	-3              
1 0 0 0 0	-4              

1 1 1 1 1        0 Robot found continuous line : STOPPED
0 0 0 0 0        0 Robot found no line: turn 180o

1 1 1 0 0       sharp turns - need previous error to
0 1 1 1 0       determine direction of the turn
0 0 1 1 1       i set var numSensors = 3 to see this

*/
void readLFSsensors()
{
  
  //int LFS0 = analogRead(lineFollowSensor0);
  //int LFS1 = analogRead(lineFollowSensor1);
  //int LFS2 = analogRead(lineFollowSensor2);
  //int LFS3 = analogRead(lineFollowSensor3);
  //int LFS4 = analogRead(lineFollowSensor4);
     
  mode = FOLLOWING_LINE;  // for all patterns <>  (0.0.0.0.0  or  1.1.1.1.1)
  error = previousError;  // cover the patterns not originally anticipated

  LFSensor[0] = digitize(analogRead(lineFollowSensor0));
  LFSensor[1] = digitize(analogRead(lineFollowSensor1));
  LFSensor[2] = digitize(analogRead(lineFollowSensor2));
  LFSensor[3] = digitize(analogRead(lineFollowSensor3));
  LFSensor[4] = digitize(analogRead(lineFollowSensor4));
  
  Serial.print("Pattern: ");
  Serial.print(LFSensor[0]);
  Serial.print(".");
  Serial.print(LFSensor[1]);
  Serial.print(".");
  Serial.print(LFSensor[2]);
  Serial.print(".");
  Serial.print(LFSensor[3]);
  Serial.print(".");
  Serial.println(LFSensor[4]);
  
  
  //numSensors = LFSensor[0] + LFSensor[1] + LFSensor[2] + LFSensor[3] + LFSensor[4];
 
  
  if((     LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -2;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -3;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -4;}

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = STOPPED; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = NO_LINE; error = 0;}
  
      Serial.print (" P: ");
     Serial.print (P);
     Serial.print (" I: ");
     Serial.print (I);
     Serial.print (" D: ");
     Serial.print (D);
     Serial.print (" PID: ");
     Serial.println (PIDvalue);

  //if (false) { //(numSensors == 3) {

  //   Kd   =   35;
  //   if (previousError < 0) {
  //     error = -5; 
  //   } else {
  //     error = 5;
  //   }
  //} else {
  //  Kd = orgKd;
  //} 
}

void xsetup() {}

//---------------------------------------------
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
  
  // line follow sensors
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);
    
  BT1.print("Check the PID constants to be sent to Robot");
  BT1.println('\n');
  
  while (!digitalRead(buttonPin) && !mode)
  {  
    checkBTcmd();  // verify if a comand is received from BT remote control
    manualCmd ();    
    command = "";  
  }
  checkPIDvalues();
}

void loop() 
{

  readLFSsensors();
  
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
      if (false) {//(saveError < errorCount) {
        saveError = saveError + 1;
        error = previousError * errorCorr;
        calculatePID();
        motorPIDcontrol();

      }
      break;

    case FOLLOWING_LINE:   
      calculatePID();
      //checkPIDvalues();
      motorPIDcontrol();   
      //testLineFollowSensors();
 
      break;     
  }
}

void xloop() 
{
 
 delay(7000);
  //adjL = 39; ok for 50//ok for power = 50
  adjL = 45; 
  power = iniMotorPower;
  
 motorForward();
 delay(10000);
 motorStop();
 
}

