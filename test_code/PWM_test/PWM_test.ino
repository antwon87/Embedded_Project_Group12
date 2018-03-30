// With 10-bit resolution, write values to use for our wheels are:
//    Fastest ccw: 51
//    Stopped: 76
//    Fastest cw: 102

#define PWM_PIN 23

void setup() {
  Serial.begin(115200);

  // Set pin 23 to output, don't think this is actually necessary
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 50);  // (pin, frequency in Hz)
  analogWriteResolution(10);          // (# of bits)
  analogWrite(PWM_PIN, 102);          // (pin, 0 .. 2^(# of bits))
}

void loop() {
  // There must be a register storing the current output value. Which is it?
  // GPIOC_PDOR bit 2? Nope.
  // Duty cycle is in FTM0_C1V 
  //   FTM0_C1V[15:0] = 59941 when 1023/1024 duty cycle
  //                    30000 when 50% duty cycle (512/1024)
  //                    29941 when 511/1024
  //                    15000 when 25% duty cycle (256/1024)
  //                    0 when 100% duty cycle  
  // Default MOD (maximum timer value) is 60000
  // FTMx_SWOCTRL bits 8-15 corresponding to channel n software output control value? Nope.
  
//  Serial.print("MOD = ");
//  Serial.println(FTM0_MOD & 0xFFFF);
//  Serial.print("Value = ");
//  Serial.println(FTM0_C1V & 0xFFFF);
//  Serial.print("Initial value = ");
//  Serial.println(FTM0_CNTIN & 0xFFFF);
//  Serial.println(FTM0_SWOCTRL, 16);

  delay(2000);
}
