#include <Bounce.h>

const int echoPin = 0;
const int trigPin = 1;
IntervalTimer trigTimer;

unsigned long cur = 0;
unsigned long lastEchoRise = 0;

void setup() {
  Serial.begin(115200);

  pinMode(echoPin, INPUT_PULLUP);
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, 0);
  
//  noInterrupts();
//  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
//  interrupts();
}

void loop() {
  digitalWrite(trigPin, HIGH);
//  cur = micros();
//  Serial.print("Set high at ");
//  Serial.println(cur);
  trigTimer.begin(trigEnd, 10);

  /* pulseIn() test */
  unsigned long echoTime = 0;
  float distance = 0;
  echoTime = pulseIn(echoPin, HIGH);
  Serial.print("Echo took ");
  Serial.print(echoTime);
  Serial.println(" microseconds to arrive.");
  distance = ((340 * ((float) echoTime / 1000000))/ 2) * 100;
//    distance = (float) echoTime / 27.6233 / 2;
  Serial.print("Distance is ");
  Serial.print(distance);
  Serial.println("cm.");
  Serial.println();
  /* End pulseIn() test */

  
  delay(2000);
}

void trigEnd() {
  cur = micros();
  digitalWrite(trigPin, LOW);
//  Serial.print("Set low at ");
//  Serial.println(cur);
//  Serial.println();
  trigTimer.end();
}

void echoISR() {
//  Serial.println("Interrupt");
  unsigned long echoTime = 0;
  float distance = 0;
  if (digitalRead(echoPin)) {
    lastEchoRise = micros();
  } else {
    echoTime = micros() - lastEchoRise;
    Serial.print("Echo took ");
    Serial.print(echoTime);
    Serial.println(" microseconds to arrive.");
    distance = ((340 * ((float) echoTime / 1000000))/ 2) * 100;
//    distance = (float) echoTime / 27.6233 / 2;
    Serial.print("Distance is ");
    Serial.print(distance);
    Serial.println("cm.");
    Serial.println();
  }
}

