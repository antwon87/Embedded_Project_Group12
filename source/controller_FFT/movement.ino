void stopCar(void) {
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);
}

void goForward(int leftSpeed, int rightSpeed) {
  analogWrite(PWM_LEFT_PIN, leftSpeed);           //6553
  analogWrite(PWM_RIGHT_PIN, rightSpeed);          // 3276
  //  delay(15);
}

void goForward(void) {
  analogWrite(PWM_LEFT_PIN, STRAIGHT_SPEED_LEFT);
  analogWrite(PWM_RIGHT_PIN, straightSpeedRight);
}

void turnLeft(float time) {
  analogWrite(PWM_LEFT_PIN, leftTurnSpeed);
  analogWrite(PWM_RIGHT_PIN, leftTurnSpeed);
  delayMicroseconds(time);
  stopCar();
}

void turnLeft(void) {
  analogWrite(PWM_LEFT_PIN, leftTurnSpeed);
  analogWrite(PWM_RIGHT_PIN, leftTurnSpeed);
}

void turnRight(float time) {
  analogWrite(PWM_LEFT_PIN, RIGHT_TURN_SPEED);
  analogWrite(PWM_RIGHT_PIN, RIGHT_TURN_SPEED);
  delayMicroseconds(time);
  stopCar();
}

void turnRight(void) {
  analogWrite(PWM_LEFT_PIN, RIGHT_TURN_SPEED);
  analogWrite(PWM_RIGHT_PIN, RIGHT_TURN_SPEED);
}

void calibrate_straight() {

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
  analogWrite(PWM_LEFT_PIN, STRAIGHT_SPEED_LEFT);
  analogWrite(PWM_RIGHT_PIN, straightSpeedRight);
  delay(2000);
  stopCar();
  delay(3000);
}

void calibrate_turn() {
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
  //  Serial.print("turnSpeed = ");
  //  Serial.println(turnSpeed);
  //  Serial.println();
  analogWrite(PWM_LEFT_PIN, leftTurnSpeed);
  analogWrite(PWM_RIGHT_PIN, leftTurnSpeed);
  delay(1000);
  stopCar();
  delay(500);
  analogWrite(PWM_LEFT_PIN, RIGHT_TURN_SPEED);
  analogWrite(PWM_RIGHT_PIN, RIGHT_TURN_SPEED);
  delay(1000);
  stopCar();
  delay(3000);
}
