// constants won't change. They're used here to set pin numbers:
const int buttonOne = 4;      // right switches pin
const int buttonTwo = 0;      // left  switches pin

// variables will change:
int buttonOneState = 0;         // variable for reading the pushbutton status
int buttonTwoState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the pushbuttons:
  pinMode(buttonOne, INPUT);
  pinMode(buttonTwo, INPUT);
  
  // Open serial monitor at 115200 baud to see ping results.
  Serial.begin(115200); 

}

void loop() {
  // read the state of the pushbuttons:
  buttonOneState = digitalRead(buttonOne);
  buttonTwoState = analogRead(buttonTwo);

  // check if a pushbutton is pressed. 
  if (buttonOneState > 0) {
    Serial.print("RIGHT BUTTON: ");
    Serial.println(buttonOneState);
  }
  
  if (buttonTwoState > 250) {
    Serial.print("LEFT BUTTON: ");
    Serial.println(buttonTwoState);
  }
  

}
