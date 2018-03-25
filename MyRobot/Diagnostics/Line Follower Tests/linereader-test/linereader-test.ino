

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


unsigned int convert(int whichPin)
{
  
  int x= analogRead(whichPin);
  Serial.print(x); Serial.print(" : ");  
  if (x > 255) {
    return 1;
    }
  else {
    return 0;
  }
    

}    
  

void setup()
{
  Serial.begin(9600);              //  setup serial
}

void loop()
{
  
  // read the analog input pins
  //linesensor1 = convert(1);
  //linesensor2 = convert(2);
  //linesensor3 = convert(3);
  //linesensor4 = convert(4);
  //linesensor5 = convert(5);
  
  linesensor3 = (analogRead(analogPin3));
  Serial.println(linesensor3);
  
  //Serial.print(linesensor1); Serial.print(" . ");
  //Serial.print(linesensor2); Serial.print(" . ");
  //Serial.print(linesensor3); Serial.print(" . ");
  //Serial.print(linesensor4); Serial.print(" . ");
  //Serial.println(linesensor5);
  
  delay(50);
}

