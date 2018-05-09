void stopCar(void) {
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);
}

void goForward(void) {
  analogWrite(PWM_LEFT_PIN, STRAIGHT_SPEED_LEFT);
  analogWrite(PWM_RIGHT_PIN, straightSpeedRight);
}

void goForward(unsigned long time) {
  analogWrite(PWM_LEFT_PIN, STRAIGHT_SPEED_LEFT);
  analogWrite(PWM_RIGHT_PIN, straightSpeedRight);
  delayMicroseconds(time);
  stopCar();
}

void turnLeft(unsigned long time) {
  analogWrite(PWM_LEFT_PIN, leftTurnSpeed);
  analogWrite(PWM_RIGHT_PIN, leftTurnSpeed);
  delayMicroseconds(time);
  stopCar();
}

void turnLeft(void) {
  analogWrite(PWM_LEFT_PIN, leftTurnSpeed);
  analogWrite(PWM_RIGHT_PIN, leftTurnSpeed);
}

void turnRight(unsigned long time) {
  analogWrite(PWM_LEFT_PIN, RIGHT_TURN_SPEED);
  analogWrite(PWM_RIGHT_PIN, RIGHT_TURN_SPEED);
  delayMicroseconds(time);
  stopCar();
}

void turnRight(void) {
  analogWrite(PWM_LEFT_PIN, RIGHT_TURN_SPEED);
  analogWrite(PWM_RIGHT_PIN, RIGHT_TURN_SPEED);
}

void calibrate_straight(void) {

  calibrateRead = analogRead(CALIBRATION_PIN);

  float adjust = (((float) calibrateRead / (779 * 10)) + 0.95);
  Serial.print("Adjust = ");
  Serial.println(adjust);
  straightSpeedRight = STRAIGHT_BASE_RIGHT * adjust;
  if (straightSpeedRight < 5210)
    straightSpeedRight = 5210;
  else if (straightSpeedRight > 6554)
    straightSpeedRight = 6554;
  Serial.print("straightSpeedRight = ");
  Serial.println(straightSpeedRight);
  Serial.println();
  goForward(2000000);
  delay(3000);
}

void calibrate_turn(void) {
  calibrateRead = analogRead(CALIBRATION_PIN);

  //  Serial.print("Value = ");
  //  Serial.println(val);
  float adjust = (((float) calibrateRead / (779 * 20)) + 0.975);
  //  Serial.print("Adjust = ");
  //  Serial.println(adjust);
  leftTurnSpeed = LEFT_TURN_SPEED_BASE * adjust;
  if (leftTurnSpeed < 5210)
    leftTurnSpeed = 5210;
  else if (leftTurnSpeed > 6554)
    leftTurnSpeed = 6554;
  Serial.print("leftTurnSpeed = ");
  Serial.println(leftTurnSpeed);
  Serial.println();
  turnLeft(1000000);
  stopCar();
  delay(500);
  turnRight(1000000);
  stopCar();
  delay(3000);
}

void evasiveManeuvers(void) {
  unsigned long time = 0;
  stopCar();
  delay(100);  // might need small delay for movement change
  turnRight(BASE_TURN_MICROS * 2.2);  // Turn right to avoid obstruction
  stopCar();
  delay(100);  // might need small delay for movement change
  time = micros();
  
  goForward();  // Move forward for some time to get away from obstruction
  while (micros() < time + (BASE_FORWARD_MICROS * 2.7)) {
    distance = ultraSonic();
    if (distance > 0 && distance < 20) {  // Make sure to dodge any obstruction in the new path.
      evasiveManeuvers();
      return;
    }
  }
  
  stopCar();
  delay(100);  // might need small delay for movement change
  toSearching(target);  // Start a new search to find the target again. 
}

/* Initiates a reading from the distance sensor and returns the
    calculated distance.
*/
unsigned int ultraSonic(void) {
  //  digitalWrite(TRIG_PIN, LOW);
  //  delayMicroseconds(2);

  unsigned long duration = 0;
  unsigned int distance = 400;

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH, 40000);

  // Calculating the distance
  distance = (duration * 0.034) / 2;

  // Prints the distance on the Serial Monitor
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print(" Distance: ");
  Serial.println(distance);

  return distance;
}
