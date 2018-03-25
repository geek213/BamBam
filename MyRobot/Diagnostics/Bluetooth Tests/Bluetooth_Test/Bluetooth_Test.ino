#include <SoftwareSerial.h>

SoftwareSerial Genotronex(3,4);

int ledpin = 13;
int BluetoothData;


void setup() {
  // put your setup code here, to run once:
  Genotronex.begin(9600);
  Genotronex.println("Bluetooth on;please press 1 or 0 blink LED...");
  pinMode(ledpin, OUTPUT);
  Serial.println("WTF???");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Genotronex.available()) {
    BluetoothData =Genotronex.read();
    if (BluetoothData== '1') {
       Genotronex.println("LED ON !");
    }
    if (BluetoothData== '0') {
       Genotronex.println("LED OFF !");
    }
  }
  delay(100);
}

