#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
int val;
int rSpeed;

void setup() {
  Serial.begin(115200);
  pinMode(A9, INPUT);
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(PWM_LEFT_PIN, 50);
  analogWriteFrequency(PWM_RIGHT_PIN, 50);
  val = 0;
//  rSpeed = 5410 * ((analogRead(A9) / (770 * 2)) + 1);
//  if (rSpeed < 5210)
//    rSpeed = 5210;
//  else if (rSpeed > 6554)
//    rSpeed = 6554;
//  analogWrite(PWM_LEFT_PIN, 4340);
//  analogWrite(PWM_RIGHT_PIN, rSpeed);
}


void loop() {
//  test_forward();
  test_turn();
  delay(2000);
}

void test_turn() {
  val = analogRead(A9);

  Serial.print("Value = ");
  Serial.println(val);
  float adjust = (((float) analogRead(A9) / (779 * 20)) + 0.975);
  Serial.print("Adjust = ");
  Serial.println(adjust);
  rSpeed = 4440 * adjust;
  if (rSpeed > 4540)
    rSpeed = 4540;
  else if (rSpeed < 3276)
    rSpeed = 3276;
  Serial.print("rSpeed = ");
  Serial.println(rSpeed);
  Serial.println();
  analogWrite(PWM_LEFT_PIN, rSpeed);
  analogWrite(PWM_RIGHT_PIN, rSpeed);
  delay(1000);
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);
  delay(500);
  analogWrite(PWM_LEFT_PIN, 5290);
  analogWrite(PWM_RIGHT_PIN, 5290);
  delay(1000);
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);
  delay(3000);
  
}

void test_forward() {
  
  val = analogRead(A9);

  Serial.print("Value = ");
  Serial.println(val);
  float adjust = (((float) analogRead(A9) / (779 * 4)) + 0.875);
  Serial.print("Adjust = ");
  Serial.println(adjust);
  rSpeed = 5410 * adjust;
  if (rSpeed < 5210)
    rSpeed = 5210;
  else if (rSpeed > 6554)
    rSpeed = 6554;
  Serial.print("rSpeed = ");
  Serial.println(rSpeed);
  Serial.println();
  analogWrite(PWM_LEFT_PIN, 4340);
  analogWrite(PWM_RIGHT_PIN, rSpeed);
  delay(2000);
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);
}

