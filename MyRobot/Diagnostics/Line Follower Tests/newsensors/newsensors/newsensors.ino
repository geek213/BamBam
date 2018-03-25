/* WaveShare Sensor Diagnostics
*/

const int buttonPin = 4;         // right switch pin (digital)


float rPosition = 0;

//PID variables...
float Kp = 5;
float Ki = 0;
float Kd = 0;

float power_error = 0;
float proportional = 0;
float last_proportional = 0;
float integral = 0;
float derivative = 0;

// sensor settings...
float in1;
float in2;
float in3;
float in4;
float in5;

float a1Min = 80;
float a2Min = 80;
float a3Min = 80;
float a4Min = 80;
float a5Min = 80;

float a1Max = 180;
float a2Max = 180;
float a3Max = 180;
float a4Max = 180;
float a5Max = 180;


int digitize(float din)
{
  if (din > 150) 
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


float normalize(float nx, float nMin, float nMax) 
{
  float ny = ((nx - nMin) * 1000) / (nMax - nMin);
  
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

  return ny;
}

float weight(float wval1, float wval2, float wval3, float wval4, float wval5) 
{
  float wy = ( 1 * wval1 + 1000 * wval2 + 2000 * wval3 + 3000 * wval4 + 4000 * wval5) / (wval1 + wval2 + wval3 + wval4 + wval5);
  
  Serial.print("Weights -> ");
  Serial.print(wy);
  Serial.print(" = (1 * ");
  Serial.print(wval1);
  Serial.print(" + 1000 * ");
  Serial.print(wval2);
  Serial.print(" + 2000 * ");
  Serial.print(wval3);
  Serial.print(" + 3000 * ");
  Serial.print(wval4);
  Serial.print(" + 4000 * ");
  Serial.print(wval5);
  Serial.print(") / (");
  Serial.print(wval1);
  Serial.print(" + ");
  Serial.print(wval2);
  Serial.print(" + ");
  Serial.print(wval3);
  Serial.print(" + ");
  Serial.print(wval4);
  Serial.print(" + ");
  Serial.print(wval5);
  Serial.println(")");

  return wy;
}



void setup()
{
  Serial.begin(115200);
  Serial.println("TRSensor example");
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  // initialize the right button
  pinMode(buttonPin, INPUT);
  
   // wait until button is pushed again - signalling training start
  while (!digitalRead(buttonPin))
  {  
    
    // set sensor limits
    in1 = analogRead(A1);
    in2 = analogRead(A2);
    in3 = analogRead(A3);
    in4 = analogRead(A4);
    in5 = analogRead(A5);
    
    if (in1 < a1Min) { a1Min = in1; }
    if (in1 > a1Max) { a1Max = in1; }    
    if (in2 < a2Min) { a2Min = in2; }
    if (in2 > a2Max) { a2Max = in2; }
    if (in3 < a3Min) { a3Min = in3; }
    if (in3 > a3Max) { a3Max = in3; }    
    if (in4 < a4Min) { a4Min = in4; }
    if (in4 > a4Max) { a4Max = in4; }    
    if (in5 < a5Min) { a5Min = in5; }
    if (in5 > a5Max) { a5Max = in5; }        
  }


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

  delay(500);
  
  // train until button is pushed again - signalling training finished
  while (!digitalRead(buttonPin))
  {  
      
  }
  
} //end setup

void sensorRead()
{
  in1 = analogRead(A1);
  in2 = analogRead(A2);
  in3 = analogRead(A3);
  in4 = analogRead(A4);
  in5 = analogRead(A5);
 
  // implement min and max values recorded during training...
  if (in1 < a1Min) { in1 = a1Min; }
  if (in2 < a2Min) { in2 = a2Min; }
  if (in3 < a3Min) { in3 = a3Min; }
  if (in4 < a4Min) { in4 = a4Min; }
  if (in5 < a5Min) { in5 = a5Min; }
  
  if (in1 > a1Max) { in1 = a1Max; }
  if (in2 > a2Max) { in2 = a2Max; }
  if (in3 > a3Max) { in3 = a3Max; }
  if (in4 > a4Max) { in4 = a4Max; }
  if (in5 > a5Max) { in5 = a5Max; } 
}

void loop()
{

  sensorRead();
  rPosition = weight( normalize(in1,a1Min,a1Max), normalize(in2,a2Min,a2Max), normalize(in3,a3Min,a3Max), normalize(in4,a4Min,a4Max), normalize(in5,a5Min,a5Max) );
   
  Serial.print("Pattern : ");
  Serial.print(digitize(in1));
  Serial.print(" : ");
  Serial.print(digitize(in2));
  Serial.print(" : ");
  Serial.print(digitize(in3));
  Serial.print(" : ");
  Serial.print(digitize(in4));
  Serial.print(" : ");
  Serial.print(digitize(in5));
  Serial.println(""); 
 
  proportional = rPosition - 2000;
  derivative = proportional - last_proportional;
  integral += proportional;

  last_proportional = proportional;

  power_error = proportional * Kp + integral * Ki + derivative * Kd;

  delay(200);
 
}  //end loop
