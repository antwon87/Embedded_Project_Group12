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

#define PWM_PIN 3

void setup() {
  Serial.begin(115200);

  // Set pin 23 to output, don't think this is actually necessary
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 15000);  // (pin, frequency in Hz)
  analogWriteResolution(10);          // (# of bits)
  analogWrite(PWM_PIN, 512);          // (pin, 0 .. 2^(# of bits))
}

void loop() {

  Serial.print("FTM1_C0V[15:0] = ");
  Serial.println(FTM1_C0V & 0xFFFF);
  Serial.print("FTM1_MOD[15:0] = ");
  Serial.println(FTM1_MOD & 0xFFFF);
  Serial.print("FTM1_CNT[15:0] = ");
  Serial.println(FTM1_CNT & 0xFFFF);
  Serial.print("FTM1_OUTMASK[0] = ");
  Serial.println(FTM1_OUTMASK & 1);
  Serial.print("FTM1_CH0 = ");
  Serial.println();
  delay(2000);

  /* These two for loops go through the full range at 16-bit resolution */
//  for (int i = 3276; i <= 6554; ++i) {
//    analogWrite(PWM_PIN, i);
//    delay(15);
//  }
//  for (int i = 6553; i >= 3276; --i) {
//    analogWrite(PWM_PIN, i);
//    delay(15);
//  }
}
