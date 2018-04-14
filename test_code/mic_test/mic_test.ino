#define PWM_PIN 3

#include <Audio.h>

AudioInputAnalog adc1;
AudioAnalyzeToneDetect f1;
AudioAnalyzeToneDetect f2;
AudioAnalyzeToneDetect f3;
AudioAnalyzePeak peak;

AudioConnection patchCord1(adc1, f1);
AudioConnection patchCord2(adc1, peak);
AudioConnection patchCord3(adc1, f2);
AudioConnection patchCord4(adc1, f3);

void setup() {
  Serial.begin(115200);
  AudioMemory(12);

  // Set pin 23 to output, don't think this is actually necessary
  pinMode(PWM_PIN, OUTPUT);
  //  analogWriteFrequency(PWM_PIN, 5000);  // (pin, frequency in Hz)
  //  analogWriteResolution(10);          // (# of bits)
  //  analogWrite(PWM_PIN, 512);          // (pin, 0 .. 2^(# of bits))

  f1.frequency(8000);
  f2.frequency(5000);
  f3.frequency(3000);

}

void loop() {
  Serial.println();

  if (f1.available()) {
    Serial.print("f1Val = ");
    Serial.println(f1.read(), 4);
  }

  if (f2.available()) {
    Serial.print("f2Val = ");
    Serial.println(f2.read(), 4);
  }

  if (f3.available()) {
    Serial.print("f3Val = ");
    Serial.println(f3.read(), 4);
  }

  if (peak.available()) {
    Serial.print("Peak value: ");
    Serial.println(peak.read());
  }


  delay(500);

}
