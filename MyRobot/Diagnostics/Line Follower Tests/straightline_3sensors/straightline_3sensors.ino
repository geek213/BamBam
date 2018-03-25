#include <NewPing.h>

// configure ultrasonic rangefinger pins...
#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_REV 11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN_FWD 13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


int mode = 0;

# define STOPPED 0
# define FOLLOWING_LINE_FWD 1
# define FOLLOWING_LINE_REV 3
# define NO_LINE 2


int lastError = 0;

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


const int FORWARD = 1;
const int REVERSE = 2;
int DIRECTION = 0;

int LFSensor[5]={0, 0, 0, 0, 0};

// PID controller
float Kp=50;
float Ki=0;
float Kd=0;

int error = 0;
float P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;



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
const int buttonR = 2;     // the number of the right switches pin
const int buttonL = 3;     // the number of the left  switches pin

//const int adjustLeftMotorsFwd = 75;  // left motors are weaker than right
//const int adjustLeftMotorsRev = 15;   // not so much adj needed in reverse
const int adjustLeftMotorsFwd = 47;  // left motors are weaker than right
const int adjustLeftMotorsRev = 0;   // not so much adj needed in reverse


// variables will change:
int buttonRState = 0;       // variable for reading the pushbutton status
int buttonLState = 0;       // variable for reading the pushbutton status

//
int analogPin1 = 1;    
int analogPin2 = 2; 
int analogPin3 = 3;
int analogPin4 = 4; 
int analogPin5 = 5;

//
int linesensor1 = 0; 
int linesensor2 = 0; 
int linesensor3 = 0; 
int linesensor4 = 0;
int linesensor5 = 0; 


// NewPing setup of pins and maximum distance.
NewPing sonarFwd(TRIGGER_PIN, ECHO_PIN_FWD, MAX_DISTANCE); 

// NewPing setup of pins and maximum distance.
NewPing sonarRev(TRIGGER_PIN, ECHO_PIN_REV, MAX_DISTANCE); 


unsigned int pingDelay(int duration, int pingdir)
{
  int p = 0;
  
  for (int i = 1; i < duration / 75; i++) {
    if (pingdir == 1) {
      p = sonarFwd.ping_cm();  
      Serial.println("Ping Forward Sensor");
    }
    else {
      p = sonarRev.ping_cm(); 
      Serial.println("Ping Reverse Sensor");
    } 
    Serial.print("Ping: ");
    Serial.print(p);          // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.print(" Dir: ");
    Serial.print(pingdir);    // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm ");
    
    if (p > 0 and p < 50) {
        // turn off motors!
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);  
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
     // Serial.println("*** STOPPED ***");
    }
    delay(50);  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings. 
  } 
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
  // read the LINE SENSOR pins

  LFSensor[0] = analogRead(lineFollowSensor0); 
  LFSensor[1] = analogRead(lineFollowSensor1); 
  LFSensor[2] = analogRead(lineFollowSensor2); 
  LFSensor[3] = analogRead(lineFollowSensor3);
  LFSensor[4] = analogRead(lineFollowSensor4); 

  Serial.print(LFSensor[0]); Serial.print(" . ");
  Serial.print(LFSensor[1]); Serial.print(" . ");
  Serial.print(LFSensor[2]); Serial.print(" . ");
  Serial.print(LFSensor[3]); Serial.print(" . ");
  Serial.print(LFSensor[4]); Serial.print(" ");
  Serial.println();
  
  // Tweaking: Try to stay on track with only THREE sensors????
  LFSensor[0] = 0; //convert(LFSensor[0]);
  LFSensor[1] = convert(LFSensor[1]);
  LFSensor[2] = convert(LFSensor[2]);
  LFSensor[3] = convert(LFSensor[3]);
  LFSensor[4] = 0; //convert(LFSensor[4]);
  
  lastError = error;
  
  //Serial.print(LFSensor[0]); Serial.print(" . ");
  //Serial.print(LFSensor[1]); Serial.print(" . ");
  //Serial.print(LFSensor[2]); Serial.print(" . ");
  //Serial.print(LFSensor[3]); Serial.print(" . ");
  //Serial.print(LFSensor[4]); Serial.print(": ");
  
 if (DIRECTION == FORWARD) {
  mode = FOLLOWING_LINE_FWD;
  if((     LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 5;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 6;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 7;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 8;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = STOPPED; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = NO_LINE; error = 0;}
  Serial.print("DIRECTION = FORWARD; Error = "); 

 } else {
  mode = FOLLOWING_LINE_REV;
  if((     LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {error = 8;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {error = 7;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {error = 6;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {error = 5;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 2;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 3;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {error = 4;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = STOPPED; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = NO_LINE; error = 0;}
  Serial.print("DIRECTION = REVERSE; Error = "); 

 }
  
  Serial.println(error); 
}


void varySpeed(int startSpeed, int endSpeed)
{
  // this function will run the motors across the range of possible speeds
  // note that maximum speed is determined by the motor itself and the operating voltage
  // the PWM values sent by analogWrite() are fractions of the maximum speed possible 
  // by your hardware
  
  // *** this function assumes :
  // ***    robot is already in motion ***
  // ***    robot will be stopped by the calling funtion ***
  
  if (endSpeed > startSpeed) {    // accelerate to endSpeed
    for (int i = startSpeed; i < endSpeed; i++)
    {
      analogWrite(enR, i);  
      analogWrite(enL, i); //+43);  // fine-tune LEFT wheels with a little more power
      delay(20);
    }
  }
  
  if (endSpeed < startSpeed) {    // decelerate from maximum speed to zero
    for (int i = endSpeed; i >= startSpeed; --i)
    {
      analogWrite(enR, i);
      analogWrite(enL, i); //+5);
      delay(20);
    }
  } 
}

void driveForward(int duration, int speed){ // this function will run the motors in REVERSE at a specified speed
  
  // turn on right motors
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // turn on left motors
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  // set motors to speed in possible range 0~255
  analogWrite(enR, speed);
  analogWrite(enL, speed + adjustLeftMotorsFwd);

  // accellerate to desired speed
  //varySpeed(75, speed);  

  Serial.println("DRIVING FORWARD");
  
  // run the motors for this duration
  delay(duration);
  //pingDelay(duration, 1);

  // decellerate to desired speed
  //varySpeed(speed, 75);  
    
  // now turn off motors ???
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, LOW);  
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, LOW);
}


/////////////////////////////////////////////////////////

void followLineForward() 
{ 
  int RightMotors = 0;
  int LeftMotors = 0;
  
  // turn on right motors
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // turn on left motors
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  int k = 84;
  int mult = 0;
  int d = 0;

  Serial.println("FOLLOWING LINE FORWARD");
  
  switch (error)
  {
    case 4: 
      RightMotors =  k + (error * 15);
      LeftMotors =  k;
      break;

    case 3: 
      RightMotors =  k + (error * 15);
      LeftMotors =   k;
      break;

    case 2: 
      RightMotors =  k + 6; //(error * 40); // was 15
      LeftMotors =   k - 4;
      d=80;
      break;

    case 1: 
      if (lastError == 0) {
        RightMotors =  k + 11; //(error * 25); // was 15
        LeftMotors =   k - 6;
      } else {
        RightMotors = k - 5;
        LeftMotors =  k - 5;
      }
      break;

    case 0:  
      RightMotors = k;
      LeftMotors =  k;
      break;

    case 5:
      if (lastError == 0 ) { 
        mult = error - 4;
        RightMotors = k - 6;
        LeftMotors =  k + 10; //(mult * 25);  // was 15
      } else {
        RightMotors = k - 5;
        LeftMotors =  k - 5;
      }
      break;  
  
    case 6:  
      mult = error - 4;
      RightMotors = k - 5;
      LeftMotors =  k + 7; //(mult * 40);  // was 15
      d = 80;
      break;  

    case 7:  
      mult = error - 4;
      RightMotors = k;
      LeftMotors =  k + (mult * 15);
      break;  

    case 8:  
      mult = error - 4;
      RightMotors = k;
      LeftMotors =  k + (mult * 15);
      break;  
   }

  LeftMotors = LeftMotors + adjustLeftMotorsFwd;
  if (LeftMotors > 255) {
      LeftMotors = 255;
  }
  analogWrite(enR, RightMotors);
  analogWrite(enL, LeftMotors);
  
  if (d > 0) {
    //delay(d);
  }
  
  Serial.print("Adj = ");
  Serial.print(error);
  Serial.print(": Right: ");
  Serial.print(RightMotors);
  Serial.print(": Left: ");
  Serial.println(LeftMotors);

}

//////////////////////////////////////////////////////////////

void followLineReverse() 
{ 
  int RightMotors = 0;
  int LeftMotors = 0;
  
  // turn on right motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // turn on left motors
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  int k = 80;
  int mult = 0;
  //int d = 50;
  
  
  Serial.println("FOLLOWING LINE REVERSE");

  switch (error)
  {
    case 4: 
      RightMotors =  k + (error * 15);
      LeftMotors =  k;
      break;

    case 3: 
      RightMotors =  k + (error * 15);
      LeftMotors =  k;
      break;

    case 2: 
      RightMotors =  k + (error * 40);
      LeftMotors =  k - 20;
      break;

    case 1: 
      RightMotors =  k + (error * 25);
      LeftMotors =  k - 10;
      break;

    case 0:  
      RightMotors = k;
      LeftMotors = k;
      break;

    case 5:  
      mult = error - 4;
      RightMotors = k - 10;
      LeftMotors =  k + (mult * 25);
      break;  
  
    case 6:  
      mult = error - 4;
      RightMotors = k - 20;
      LeftMotors =  k + (mult * 40);
      break;  

    case 7:  
      mult = error - 4;
      RightMotors = k;
      LeftMotors =  k + (mult * 15);
      break;  

    case 8:  
      mult = error - 4;
      RightMotors = k;
      LeftMotors =  k + (mult * 15);
      break;  
   }

  int x = RightMotors;
  RightMotors = LeftMotors;
  LeftMotors = x;
  
  LeftMotors = LeftMotors + adjustLeftMotorsRev;
  if (LeftMotors > 255) {
      LeftMotors = 255;
  }
  analogWrite(enR, RightMotors);
  analogWrite(enL, LeftMotors);
  
  //delay(d);
  
  Serial.print("Adj = ");
  Serial.print(error);
  Serial.print(": Right: ");
  Serial.print(RightMotors);
  Serial.print(": Left: ");
  Serial.println(LeftMotors);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void driveReverse(int duration, int speed){ // this function will run the motors in FORWARD at a specified speed
  
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // set motors to speed in possible range 0~255
  analogWrite(enR, speed); 
  analogWrite(enL, speed + adjustLeftMotorsRev);
  
  // accellerate to desired speed
  //varySpeed(75, speed);  
 
  Serial.println("DRIVING REVERSE");

  // run the motors for this duration
  delay(duration);
  //pingDelay(duration, 2);

  // decellerate to indicated speed
  //varySpeed(speed, 75);  
   
  // now turn off motors ??
  // digitalWrite(in1, LOW);
  // digitalWrite(in2, LOW);  
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, LOW);
}


void spinRight(int duration, int speed)
{
  // this function will run the left motors in reverse at a fixed speed
  // and the right motors forward to attempt a 90 degree left turn
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // set motor A speed to 100 out of possible range 0~255
  analogWrite(enR, speed);

  // set motor B speed to 100 out of possible range 0~255
  analogWrite(enL, speed);
  
  // spin this duration
  delay(duration);
  
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void spinLeft(int duration, int speed)
{
  // this function will run the left motors in reverse at a fixed speed
  // and the right motors forward to attempt a 90 degree left turn
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  // set motor A speed to 100 out of possible range 0~255
  analogWrite(enR, speed);

  // set motor B speed to 100 out of possible range 0~255
  analogWrite(enL, speed);
  
  // spin this duration
  delay(duration);
  
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// TESTING NEW FUNCTIONS
void turnRight(int duration)
{
  // run the right motors slower than the left motors to facilitate a left turn

  // turn on left motors  -> Forward
  digitalWrite(in1, HIGH);  
  digitalWrite(in2, LOW);   
  
  // turn on right motors -> Forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  // set left motors in high speed 
  analogWrite(enL, 200);

  // set right motors in slow speed
  analogWrite(enR, 50);
  
  // turn this duration
  delay(duration);
  
  // stop motors - for now
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void motorStop()
{
  // stop motors - for now
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}


void turnLeft(int duration)
{
  // this function will run the left motors slower than the
  // right motors forward to facilitate a left turn

  // turn on left motors  -> Forward
  digitalWrite(in1, HIGH);  
  digitalWrite(in2, LOW);   
  
  // turn on right motors -> Forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  // set right motors in high speed 
  analogWrite(enR, 200);

  // set left motors in slow speed
  analogWrite(enL, 50);
  
  // turn this duration
  delay(duration);
  
  // stop motors - for now
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setup() {  
  // set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // initialize the pushbuttons:
  pinMode(buttonR, INPUT);
  pinMode(buttonL, INPUT);
  
  DIRECTION = FORWARD;
 
  // Open serial monitor at 115200 baud to see ping results.
  Serial.begin(115200); 
}

void loop() {
  
  //delay(1000);                
  //spinRight(750, 150);      // duration (in milliseconds) and speed (0 to 255)
  //delay(500);                
  //spinLeft(750, 150);       // duration (in milliseconds) and speed (0 to 255)
  
//  delay(3000);             
//  driveReverse(4000,100);     // duration (in milliseconds) and speed (0 to 255)
  
  /////////////////////////////
  
  // read the state of the pushbuttons:
  buttonRState = digitalRead(buttonR);
  buttonLState = digitalRead(buttonL);
  
  readLFSsensors(); 
  
  switch (mode)
  {
    case STOPPED: 
      motorStop();
      // BT1.print("The End");
      // ledBlink();
      previousError = error;
      break;

    case FOLLOWING_LINE_FWD:  
      DIRECTION = FORWARD;
      followLineForward();
      
      //calculatePID();
      //motorPIDcontrol();    
      break;  
   
    case FOLLOWING_LINE_REV:  
      DIRECTION = REVERSE;
      followLineReverse();
      
      //calculatePID();
      //motorPIDcontrol();    
      break;  
      
    case NO_LINE:
      motorStop();
      Serial.println("NO LINE!!!"); 
      if (false) {
      if (DIRECTION == FORWARD) {  
        DIRECTION = REVERSE;
        mode = FOLLOWING_LINE_REV;
        driveReverse(1500, 90);
        //followLineReverse();
      } else {
        DIRECTION = FORWARD;
        mode = FOLLOWING_LINE_FWD;
        driveForward(1000, 90);
        //followLineForward();
      }
      }
      
      break;

   
   }
  //delay(50);   

}  


