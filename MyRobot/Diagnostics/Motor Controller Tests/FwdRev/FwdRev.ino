#include <NewPing.h>

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
const int buttonR = 2;    // the number of the right switches pin
const int buttonL = 3;     // the number of the left  switches pin

int adjustLeftMotorsFwd = 60;  // left motors are weaker than right
int adjustLeftMotorsRev = 10;   // not so much adj needed in reverse

// variables will change:
int buttonRState = 0;       // variable for reading the pushbutton status
int buttonLState = 0;       // variable for reading the pushbutton status

// NewPing setup of pins and maximum distance.
NewPing sonarFwd(TRIGGER_PIN, ECHO_PIN_FWD, MAX_DISTANCE); 

// NewPing setup of pins and maximum distance.
NewPing sonarRev(TRIGGER_PIN, ECHO_PIN_REV, MAX_DISTANCE); 


unsigned int pingDelay(int duration, int pingdir)
{
  int p = 0;
  
  // for (int i = 1; i < duration / 75; i++) {
  for (int i = 1; i < duration / 50; i++) {
    if (pingdir == 1) {
      p = sonarFwd.ping_cm();  
      Serial.print("Ping Forward Sensor: ");
      Serial.println(p);
    }
    else {
      p = sonarRev.ping_cm(); 
      Serial.print("Ping Reverse Sensor: ");
      Serial.println(p);
    } 
    Serial.print("Ping: ");
    Serial.print(p);            // Send ping, get distance in cm and print result (0 = out of range)
    Serial.print("cm; Dir: ");
    Serial.println(pingdir);    
    
    if (p > 0 and p < 50) {
        // turn off motors!
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);  
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        Serial.println("*** STOPPED ***");
    }
    delay(50);  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings. 
  } 
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
      analogWrite(enL, (i + adjustLeftMotorsFwd)); // fine-tune LEFT wheels with more power
      delay(20);
    }
  }
  
  if (endSpeed < startSpeed) {    // decelerate from maximum speed to zero
    for (int i = endSpeed; i >= startSpeed; --i)
    {
      analogWrite(enR, i);
      analogWrite(enL, (i + adjustLeftMotorsRev)); // fine-tune LEFT wheels with more power
      delay(20);
    }
  } 
}

void driveForward(int duration, int speed){ 
  // this function will run the motors in REVERSE at a specified speed
  
  // turn right motors on
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // turn left motors on
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  // set motors to speed in possible range 0~255
  analogWrite(enR, speed);
  analogWrite(enL, (speed + adjustLeftMotorsFwd));

  // accellerate to desired speed
  // varySpeed(75, speed);  

  Serial.println("FORWARD");
  
  // run the motors for this duration
  //delay(duration);
  pingDelay(duration, 1);

  // decellerate to desired speed
  // varySpeed(speed, 75);  
    
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


void driveReverse(int duration, int speed){ 
  // this function will run the motors in FORWARD at a specified speed
  
  // turn on right motors - reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  // turn on left motors - reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // set motors to speed in possible range 0~255
  analogWrite(enR, speed); 
  analogWrite(enL, (speed + adjustLeftMotorsRev));
  
  // accellerate to desired speed
  // varySpeed(75, speed);  
 
  Serial.println("REVERSE");

  // run the motors for this duration
  // delay(duration);
  pingDelay(duration, 2);

  // decellerate to indicated speed
  // varySpeed(speed, 75);  
   
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
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
  
  // set right motors in slow speed
  analogWrite(enR, 50);
 
   // set left motors in high speed 
  analogWrite(enL, (200 + adjustLeftMotorsFwd));
 
  // turn this duration
  delay(duration);
  
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
  analogWrite(enL, (50 + adjustLeftMotorsFwd));
 
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
  
 
  // Open serial monitor at 115200 baud to see ping results.
  Serial.begin(115200); 
  
  //straight line tweaks: fwd and rev
  adjustLeftMotorsFwd= 75;
  adjustLeftMotorsRev= 15;
  
}

void loop() {
 
  delay(5000);                // delay 5 seconds
  driveForward(3000, 150);     // duration (in milliseconds) and speed (0 to 255) 
  driveReverse(3000, 150);
  
 
}  

