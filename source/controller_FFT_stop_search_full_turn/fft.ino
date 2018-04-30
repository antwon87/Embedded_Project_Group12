void fftSample(void) {
  double reading = 0;

  for (int i = 0; i < samples; i++) {
    microseconds = micros();    //Overflows after around 70 minutes!

    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    while (micros() < (microseconds + sampling_period_us)) {
      //empty loop
    }
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  // AVG_NUMBER point moving average for all frequencies of interest
  for (int i = 0; i < 10; ++i) {
    reading =
      vReal[freqToIndex((i + (F1 / 1000)) * 1000) - 1] +
      vReal[freqToIndex((i + (F1 / 1000)) * 1000) + 1] +
      vReal[freqToIndex((i + (F1 / 1000)) * 1000)];
//    reading = reading * 10;
    avgSum[i] -= avgHistory[i][avgPos[i]];
    avgSum[i] += reading;
    avgHistory[i][avgPos[i]] = reading;
    avgPos[i] = (avgPos[i] == AVG_NUMBER - 1) ? 0 : avgPos[i] + 1;
    magnitudes[i] = avgSum[i] / AVG_NUMBER;
  }

  // Moving average for the target. New values only added to average if above threshold.
  // This is meant to handle the buzzers only being on half the time. When the buzzer is
  // off, the value will simply be ignored and the average won't change.
  if ((vReal[tarFFTindex - 1] + vReal[tarFFTindex + 1] + vReal[tarFFTindex]) > targetThreshold[(target / 1000) - (F1 / 1000)]) {
    tarAvgSum -= tarAvgHistory[tarAvgPos];
    reading =
      vReal[tarFFTindex - 1] +
      vReal[tarFFTindex + 1] +
      vReal[tarFFTindex];
//    reading = reading * 10;
    tarAvgSum += reading;
    tarAvgHistory[tarAvgPos] = reading;
    tarAvgPos = (tarAvgPos == AVG_NUMBER - 1) ? 0 : tarAvgPos + 1;
    tarMag = tarAvgSum / AVG_NUMBER;
  }

  /* Test code for displaying moving average output */
  Serial.print("Target = ");
  Serial.print(target);
  Serial.print("   ");
  for (int i = 0; i < 10; ++i) {
    Serial.print(magnitudes[i], 3);
    Serial.print("    ");
  }
  Serial.println(tarMag);
}

int freqToIndex(int f) {
  return round((f * samples) / samplingFrequency);
}
