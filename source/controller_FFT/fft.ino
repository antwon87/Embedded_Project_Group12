void fftSample(void) {
  // FFT sampling
  //  tarMag = 0;
  //  int validSamples = 0;  // Counting a sample as valid if the target buzzer is on. This is my idea for dealing with 25% buzzer on time.
  //  while (validSamples < 10) {
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
    //    if (vReal[freqToIndex((i + 1) * 1000)] > threshold[i]) {  // value chosen arbitrarily. Needs testing.
    //    validSamples++;
    //      Serial.print("measured: ");
    //      Serial.println(vReal[tarFFTindex]);
    //    tarMag = /*vReal[tarFFTindex - 1] + */vReal[tarFFTindex]/* + vReal[tarFFTindex + 1]*/;

    avgSum[i] -= avgHistory[i][avgPos[i]];
    avgSum[i] += vReal[freqToIndex((i + 1) * 1000)];
    avgHistory[i][avgPos[i]] = vReal[freqToIndex((i + 1) * 1000)];
    avgPos[i] = (avgPos[i] == AVG_NUMBER - 1) ? 0 : avgPos[i] + 1;
    magnitudes[i] = avgSum[i] / AVG_NUMBER;
    //      if (target == (i + 1) * 1000) {
    //        tarMag = magnitudes[i];
    //      }
    //    }
  }

  // Moving average for the target. New values only added to average if above threshold.
  // This is meant to handle the buzzers only being on half the time. When the buzzer is
  // off, the value will simply be ignored and the average won't change.
  if (vReal[tarFFTindex] > threshold[(target / 1000) - 1]) {
    tarAvgSum -= tarAvgHistory[tarAvgPos];
    tarAvgSum += vReal[tarFFTindex];
    tarAvgHistory[tarAvgPos] = vReal[tarFFTindex];
    tarAvgPos = (tarAvgPos == AVG_NUMBER - 1) ? 0 : tarAvgPos + 1;
    tarMag = tarAvgSum / AVG_NUMBER;
  }
}

int freqToIndex(int f) {
  return round((f * samples) / samplingFrequency);
}
