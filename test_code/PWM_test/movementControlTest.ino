// Use 50Hz frequency for wheel control
// With 10-bit resolution, write values to use for our wheels are:
//    Fastest ccw: 51
//    Stopped: 71-81
//    Fastest cw: 102

// With 16-bit resolution:
//    Fastest ccw: 3276
//    Stopped: ~4540-5210
//    Fastest cw: 6554
//
//    cw is forwards for right wheel, backwards for left wheel

#define PWM_LEFT_PIN 3
#define PWM_RIGHT_PIN 4

void setup() {
  Serial.begin(115200);

  // Set pin 23 to output, don't think this is actually necessary
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  analogWriteFrequency(PWM_LEFT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteFrequency(PWM_RIGHT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteResolution(16);          // (# of bits)
  analogWrite(PWM_LEFT_PIN, 5210);          // (pin, 0 .. 2^(# of bits))
  analogWrite(PWM_RIGHT_PIN, 5210);          // (pin, 0 .. 2^(# of bits))
  Serial.println("Setup");
}

void loop() {

//  Serial.print("FTM1_C0V[15:0] = ");
//  Serial.println(FTM1_C0V & 0xFFFF);
//  Serial.print("FTM1_MOD[15:0] = ");
//  Serial.println(FTM1_MOD & 0xFFFF);
//  Serial.print("FTM1_CNT[15:0] = ");
//  Serial.println(FTM1_CNT & 0xFFFF);
//  Serial.print("FTM1_OUTMASK[0] = ");
//  Serial.println(FTM1_OUTMASK & 1);
//  Serial.println();
//  delay(2000);

  goForward();
  Serial.println("GO FORWARD");
  delay(1000);
  stopCart();
  Serial.println("STOP");
  delay(3000);
  turnLeft();
  delay(1000);
  stopCart();

}

void goForward(int leftSpeed, int rightSpeed) {
  analogWrite(PWM_LEFT_PIN, leftSpeed);           //6553
  analogWrite(PWM_RIGHT_PIN, rightSpeed);          // 3276
  delay(15);
}

void goForward() {
  analogWrite(PWM_LEFT_PIN, 5570);           //6553
  analogWrite(PWM_RIGHT_PIN, 4240);          // 3276
  delay(15);
}

void goBack(int leftSpeed, int rightSpeed) {
  analogWrite(PWM_LEFT_PIN, leftSpeed);           //3276
  analogWrite(PWM_RIGHT_PIN, rightSpeed);          //6553
  delay(15);
}

void stopCart() {
  analogWrite(PWM_LEFT_PIN, 5000);          // (pin, 0 .. 2^(# of bits))
  analogWrite(PWM_RIGHT_PIN, 5000);          // (pin, 0 .. 2^(# of bits))
  delay(15);
}

// TODO: adjust the value
void turnLeft() {
  analogWrite(PWM_LEFT_PIN, 5570);          
  analogWrite(PWM_RIGHT_PIN, 5570);      
}

// TODO: adjust the value
void turnRight() {
  analogWrite(PWM_LEFT_PIN, 5210);          
  analogWrite(PWM_RIGHT_PIN, 5210);  
  delay(15);
}


