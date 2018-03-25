

// lineFollowSensor0
int analogPin1 = 1;    
int analogPin2 = 2; 
int analogPin3 = 3;
int analogPin4 = 4; 
int analogPin5 = 5;

// LFSensor[]
int linesensor1 = 0; 
int linesensor2 = 0; 
int linesensor3 = 0; 
int linesensor4 = 0;
int linesensor5 = 0; 


unsigned int digitize(int thisValue)
{  
  if (thisValue > 255) {
    return 1;
    }
  else {
    return 0;
  }
}    
  

void setup()
{
  Serial.begin(115200);              //  setup serial
}

void loop()
{
  
  // read the analog input pins
  linesensor1 = digitize(analogRead(analogPin1));
  linesensor2 = digitize(analogRead(analogPin2));
  linesensor3 = digitize(analogRead(analogPin3));
  linesensor4 = digitize(analogRead(analogPin4));
  linesensor5 = digitize(analogRead(analogPin5));
  
  Serial.print(linesensor1); Serial.print(" . ");
  Serial.print(linesensor2); Serial.print(" . ");
  Serial.print(linesensor3); Serial.print(" . ");
  Serial.print(linesensor4); Serial.print(" . ");
  Serial.println(linesensor5);
  
  delay(50);
}

