
/*------------------------------------------------------------------
Smart Robot - Line Follower with programable PID controller via BT
==> Basic movement based on Nano Mouse Robot, developed by Michael Backus (http://www.akrobotnerd.com/ )
==> Line follow based on http://samvrit.tk/tutorials/pid-control-arduino-line-follower-robot/?ckattempt=1

Marcelo Jose Rovai - 06 April, 2016 - Visit: http://mjrobot.org
-------------------------------------------------------------------*/

//#include <Servo.h>
//#include "robotDefines.h"

String command;
String device;

// BT Module
#include <SoftwareSerial.h>
//SoftwareSerial BT1(3, 4); // El pin 10 es Rx y el pin 11 es Tx
SoftwareSerial Genotronex(3,4);


/////////////////////////////////////////////////////////////////////

int mode = 0;

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2

const int power = 500;
const int iniMotorPower = 250;
const int adj = 1;
float adjTurn = 8;

const int ledPin = 13;
const int buttonPin = 9;

// LFSensor more to the Left is "0"
const int lineFollowSensor0 = 1; 
const int lineFollowSensor1 = 2; 
const int lineFollowSensor2 = 3; 
const int lineFollowSensor3 = 4;
const int lineFollowSensor4 = 5;

int LFSensor[5]={0, 0, 0, 0, 0};

// PID controller
float Kp=50;
float Ki=0;
float Kd=0;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

#define RIGHT 1
#define LEFT -1

//Servo leftServo;
//Servo rightServo;




void ledBlink(void)
{
  
  Serial.println("ledBlink");
  
   for (int i = 0; i<4; i++)
   { 
      digitalWrite (ledPin, HIGH);
      delay (500);
      digitalWrite (ledPin, LOW);
      delay (500);
   } 
}

//-----------------------------------------------------------------------------

 void checkBTcmd()  
 { 
   //while (BT1.available())   //Check if there is an available byte to read
   while (Genotronex.available())   //Check if there is an available byte to read
   {
     delay(10); //Delay added to make thing stable 
     //char c = BT1.read(); //Conduct a serial read
     char c = Genotronex.read(); //Conduct a serial read
     device += c; //build the string.
   }  
   if (device.length() > 0) 
   {
     Serial.print("Command received from BT ==> ");
     Serial.println(device); 
     command = device;
     device ="";  //Reset the variable
     //BT1.flush();
     Genotronex.flush();
    } 
}

//------------------------------------------------------------------------
void manualCmd()
{
  switch (command[0])
  {
    case 'g':
      mode = FOLLOWING_LINE;
      break;
    
    case 's': 
      motorStop(); //turn off both motors
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
    digitalWrite (ledPin, HIGH);
    //BT1.print("Data from Arduino");
    //BT1.print(data);
    //BT1.print(" xx");
    //BT1.println('\n');
    Genotronex.print("Data from Arduino");
    Genotronex.print(data);
    Genotronex.print(" xx");
    Genotronex.println('\n');
    digitalWrite (ledPin, LOW);
}

//--------------------------------------------------------
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}

//--------------------------------------------------------
void checkPIDvalues()
{
  
  //BT1.print("PID: ");
  //BT1.print(Kp);
  //BT1.print(" - ");
  //BT1.print(Ki);
  //BT1.print(" - ");
  //BT1.println(Kd); 
  
  Genotronex.print("PID: ");
  Genotronex.print(Kp);
  Genotronex.print(" - ");
  Genotronex.print(Ki);
  Genotronex.print(" - ");
  Genotronex.println(Kd);  
  
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);  
  
}

unsigned int convert(int thisVal)
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
     int LFS0 = analogRead(lineFollowSensor0);
     int LFS1 = analogRead(lineFollowSensor1);
     int LFS2 = analogRead(lineFollowSensor2);
     int LFS3 = analogRead(lineFollowSensor3);
     int LFS4 = analogRead(lineFollowSensor4);
     
     LFS0 = convert(LFSensor[0]);
     LFS1 = convert(LFSensor[1]);
     LFS2 = convert(LFSensor[2]);
     LFS3 = convert(LFSensor[3]);
     LFS4 = convert(LFSensor[4]);


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
  
 Serial.println ("motorStop");

  //leftServo.writeMicroseconds(1500);
  //rightServo.writeMicroseconds(1500);
  //delay(200);
}

//--------------------------------------------- 
void motorForward()
{
  
    Serial.println ("motorForward");


  //leftServo.writeMicroseconds(1500 - power);
  //rightServo.writeMicroseconds(1500 + power*adj);
}

//---------------------------------------------
void motorBackward()
{
  
    Serial.println ("motorBackward");

  //leftServo.writeMicroseconds(1500 + power);
  //rightServo.writeMicroseconds(1500 - power);
}

//---------------------------------------------
void motorFwTime (unsigned int time)
{
  
    Serial.println ("motorFwTime");


  //motorForward();
  //delay (time);
  //motorStop();
}

//---------------------------------------------
void motorBwTime (unsigned int time)
{
  
  Serial.println ("motorBwTime");

  //motorBackward();
  //delay (time);
  //motorStop();
}

//------------------------------------------------
void motorTurn(int direction, int degrees)
{
  
  Serial.println ("motorTurn");

  //leftServo.writeMicroseconds(1500 - iniMotorPower*direction);
  //rightServo.writeMicroseconds(1500 - iniMotorPower*direction);
  //delay (round(adjTurn*degrees+1));
  //motorStop();
}

//---------------------------------------------------
void motorPIDcontrol()
{
  
    //Serial.println ("motorPIDcontrol");

  //int leftMotorSpeed = 1500 - iniMotorPower - PIDvalue;
  //int rightMotorSpeed = 1500 + iniMotorPower*adj - PIDvalue;
  
  // The motor speed should not exceed the max PWM value
   //constrain(leftMotorSpeed, 1000, 2000);
   //constrain(rightMotorSpeed, 1000, 2000);
  
  //leftServo.writeMicroseconds(leftMotorSpeed);
  //rightServo.writeMicroseconds(rightMotorSpeed);
  
  //Serial.print (PIDvalue);
  //Serial.print (" ==> Left, Right:  ");
  //Serial.print (leftMotorSpeed);
  //Serial.print ("   ");
  //Serial.println (rightMotorSpeed);
}

//-----------------------------
void drivePolygon(unsigned int time, int sides) // for motor test only
{
  
    Serial.println ("drivePolygon");

    //for (int i = 0; i<sides; i++)
    //{
        //motorFwTime (time);
        //motorTurn(RIGHT, 360/sides);
    //}

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
  LFSensor[0] = digitalRead(lineFollowSensor0);
  LFSensor[1] = digitalRead(lineFollowSensor1);
  LFSensor[2] = digitalRead(lineFollowSensor2);
  LFSensor[3] = digitalRead(lineFollowSensor3);
  LFSensor[4] = digitalRead(lineFollowSensor4);
  
  if((     LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error =- 1;}
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
  //BT1.begin(9600);
  Genotronex.begin(9600);
  
  pinMode(ledPin, OUTPUT);
  //pinMode(buttonPin, INPUT_PULLUP);
  
  // line follow sensors
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);
  
  // servos
  //leftServo.attach(5);
 // rightServo.attach(3);
  
  //BT1.print("check the PID constants to be sent to Robot");
  //BT1.println('\n');
  Genotronex.print("check the PID constants to be sent to Robot");
  Genotronex.println('\n');

  while (digitalRead(buttonPin) && !mode)
  {  
    checkBTcmd();  // verify if a comand is received from BT remote control
    manualCmd ();    
    command = "";  
  }
  checkPIDvalues();
  mode = STOPPED;
}

void loop() 
{
    
  //while (digitalRead(buttonPin) && !mode)
  //{ }
  
  readLFSsensors();    
    switch (mode)
  {
    case STOPPED: 
      motorStop();
      //BT.println.print("The End");
      Genotronex.println("The End");
      ledBlink();
      previousError = error;
      break;

    case NO_LINE:  
      motorStop();
      motorTurn(LEFT, 180);
      previousError = 0;
      break;

    case FOLLOWING_LINE:     
      calculatePID();
      motorPIDcontrol();    
      break;     
  }
}



