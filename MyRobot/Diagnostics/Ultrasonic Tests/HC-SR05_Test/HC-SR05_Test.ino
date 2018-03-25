// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_F   13  // Arduino pin tied to echo pin on the ultrasonic sensor in front.
#define ECHO_PIN_R   11  // Arduino pin tied to echo pin on the ultrasonic sensor at rear.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar_front(TRIGGER_PIN, ECHO_PIN_F, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
NewPing sonar_rear( TRIGGER_PIN, ECHO_PIN_R, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  delay(100);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping FWD: ");
  Serial.print(sonar_front.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("cm; Ping REV: ");
  Serial.print(sonar_rear.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
}
