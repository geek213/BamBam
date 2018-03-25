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

const int power = 110; //170;          // fwd and rev: ok = 200;
const int adjR = 0;
const int adjL = 70; //60;  // max 55
float adjRTurn = 30;
float adjLTurn = 33;

const int iniMotorPower = 100; //128; //200;   // turns:       ok = 250;

// PID controller
float Kp=10; //6; //4; //5;   //7 //15;  //50;
float Ki=0; //0;  //1 //0;
float Kd=20; //5; //2;  //3 //0


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
  
  Serial.println ("motorPIDcontrol");
  
  // right motors forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // left motors forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  
  int rightMotorSpeed = iniMotorPower + adjR;
  int leftMotorSpeed  = iniMotorPower + adjL;
  
  
  if (PIDvalue < 0)  // tripping right sensors, turn left...
  { 
    //rightMotorSpeed = rightMotorSpeed - PIDvalue;
    leftMotorSpeed  = leftMotorSpeed  -  PIDvalue;  
  }
  
  if (PIDvalue > 0)  // tripping left sensors, turn right...
  { 
    rightMotorSpeed = rightMotorSpeed + PIDvalue;
    //leftMotorSpeed  = leftMotorSpeed  + PIDvalue;  
  }
  
  // The motor speed should not exceed the max PWM value
  //constrain(rightMotorSpeed, 0, 255);
  //constrain(leftMotorSpeed,  0, 255);
  
  if (rightMotorSpeed < 0)   
    { rightMotorSpeed = 0;   }
  if (leftMotorSpeed < 0)    
    { leftMotorSpeed  = 0;   }
  if (rightMotorSpeed > 255)   
    { rightMotorSpeed = 255; }
  if (leftMotorSpeed > 255)  
    { leftMotorSpeed  = 255; }
  
  
  // set motors to speed in possible range 0~255
  analogWrite(enR, rightMotorSpeed);
  analogWrite(enL, leftMotorSpeed);
  
  Serial.print   (PIDvalue);
  Serial.print   (" ==> Left, Right:  ");
  Serial.print   (leftMotorSpeed);
  Serial.print   ("   ");
  Serial.println (rightMotorSpeed);
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
     
     int LFS0 = digitize(analogRead(lineFollowSensor0));
     int LFS1 = digitize(analogRead(lineFollowSensor1));
     int LFS2 = digitize(analogRead(lineFollowSensor2));
     int LFS3 = digitize(analogRead(lineFollowSensor3));
     int LFS4 = digitize(analogRead(lineFollowSensor4));

     Serial.print ("LFS: L  0 1 2 3 4  R ==> "); 
     Serial.print (LFS0); 
     Serial.print (" ");
     Serial.print (LFS1); 
     Serial.print (" ");
     Serial.print (LFS2); 
     Serial.print (" ");
     Serial.print (LFS3); 
     Serial.print (" ");
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
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // left motors forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    
    adjTurn = adjRTurn;
  }
  
  if (direction == LEFT) {
    
    // right motors forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
    // left motors backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    adjTurn =adjLTurn;
  }
  
  // set motors to speed
  analogWrite(enR, iniMotorPower);
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

*/
void readLFSsensors()
{
  
  //int LFS0 = analogRead(lineFollowSensor0);
  //int LFS1 = analogRead(lineFollowSensor1);
  //int LFS2 = analogRead(lineFollowSensor2);
  //int LFS3 = analogRead(lineFollowSensor3);
  //int LFS4 = analogRead(lineFollowSensor4);
     

  LFSensor[0] = digitize(analogRead(lineFollowSensor0));
  LFSensor[1] = digitize(analogRead(lineFollowSensor1));
  LFSensor[2] = digitize(analogRead(lineFollowSensor2));
  LFSensor[3] = digitize(analogRead(lineFollowSensor3));
  LFSensor[4] = digitize(analogRead(lineFollowSensor4));
  
  
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
}


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

  testLineFollowSensors();
  readLFSsensors();
  
  switch (mode)
  {
    case STOPPED: 
      motorStop();
      BT1.println("The End");
      previousError = error; 
      break;

    case NO_LINE:  
      motorStop();
      //motorTurn(LEFT, 180);
      previousError = error; //0;
      break;

    case FOLLOWING_LINE:   
      calculatePID();
      checkPIDvalues();
      motorPIDcontrol();    
      break;     
  }
}

void xloop() 
{
  
  delay(5000);
  motorFwTime(2000);
  delay(5000);


}




