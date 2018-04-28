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
//  analogWrite(PWM_LEFT_PIN, leftTurnSpeed);
//  analogWrite(PWM_RIGHT_PIN, leftTurnSpeed);
//  delay(1000);
  stopCar();
  delay(500);
  turnRight(1000000);
//  analogWrite(PWM_LEFT_PIN, RIGHT_TURN_SPEED);
//  analogWrite(PWM_RIGHT_PIN, RIGHT_TURN_SPEED);
//  delay(1000);
  stopCar();
  delay(3000);
}

void evasiveManeuvers(void) {
  stopCar();
  delay(100);  // might need small delay for movement change
  turnRight(BASE_TURN_MICROS * 1.5);  // Need to tune turn time.
  //             while (distance < 15) {  // This while won't work. Sensor stops seeing beacon before car has turned enough to avoid it.
  //               // Call distance check function if necessary
  //             }
  stopCar();
  delay(100);  // might need small delay for movement change
  goForward(BASE_FORWARD_MICROS);  // 4 seconds chosen arbitrarily. Test and tune.
  stopCar();
  delay(100);  // might need small delay for movement change
  toSearching(target);
  //      while (1);  // For testing evasion function
}
