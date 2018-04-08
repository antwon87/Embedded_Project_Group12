#define PWM_PIN 3

#include <Audio.h>

AudioInputAnalog adc1;
AudioAnalyzeToneDetect f1;
AudioAnalyzeToneDetect f2;
AudioAnalyzePeak peak;

AudioConnection patchCord1(adc1, f1);
AudioConnection patchCord2(adc1, peak);
AudioConnection patchCord3(adc1, f2);

void setup() {
  Serial.begin(115200);
  AudioMemory(12);

  // Set pin 23 to output, don't think this is actually necessary
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 3000);  // (pin, frequency in Hz)
  analogWriteResolution(10);          // (# of bits)
  analogWrite(PWM_PIN, 512);          // (pin, 0 .. 2^(# of bits))
  
  f1.frequency(3000);
  f2.frequency(3500);

}

void loop() {
  Serial.println();

  if (f1.available()) {
    Serial.print("f1Val = ");
    Serial.println(f1.read());
  }

  if (f2.available()) {
    Serial.print("f2Val = ");
    Serial.println(f2.read());
  }

  if (peak.available()) {
    Serial.print("Peak value: ");
    Serial.println(peak.read());
  }


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
