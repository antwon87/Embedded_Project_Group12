#define PWM_PIN 23;

void setup() {
  Serial.begin(115200);

  // Set pin 23 to output, don't think this is actually necessary
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 50);
  analogWriteResolution(10);
  analogWrite(PWM_PIN, 512);
}

void loop() {
  // There must be a register storing the current output value. Which is it?
  // GPIOC_PDOR bit 2? Something in FTMx_CnV? Bits 15:0 maybe? I think that is the duty cycle.
  // FTMx_SWOCTRL bits 8-15 corresponding to channel n software output control value
  Serial.println((GPIOC_PDOR >> 2) & 1U);

}
