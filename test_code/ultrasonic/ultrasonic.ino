

#define TRIG_PIN 9
#define ECHO_PIN 10
#define INT_PIN 20
#define LED_PIN 13
// defines variables
long duration;
int distance;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(INT_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  attachInterrupt(digitalPinToInterrupt(INT_PIN), ultraSonicISR, FALLING);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(INT_PIN, HIGH);
}

void loop() {
  ultraSonic();
}

void ultraSonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculating the distance
  distance= duration*0.034/2;
  
  // Prints the distance on the Serial Monitor
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print("Distance: ");
  Serial.println(distance);

  if (millis() > 2500 && distance <= 15) {
    digitalWrite(INT_PIN, LOW);
  }
}

void ultraSonicISR() {
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(INT_PIN, HIGH);
//  stopCar();
//  delay(1000);
//  goBack(1000);
//  state =FORWARD;
  
}

