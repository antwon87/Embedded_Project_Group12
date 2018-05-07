/* This code is a draft of a full control algorithm.
    The code to find the direction of a beacon is all that is
    implemented so far, but that seems to be working decently.
    There are still a couple bugs:
      When the vehicle is pointed at the beacon when the search
        starts, it will never stop searching.
      The search sometimes finishes in the wrong direction, not
        sure why yet.
*/

#include <arduinoFFT.h>

#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
#define LED_PIN       13
#define START_BUT_PIN 7
#define F1 1000
#define F2 2000
#define F3 3000
#define F4 4000
#define F5 5000
#define F6 6000
#define F7 7000
#define F8 8000
#define F9 9000
#define F10 10000

/*
  These values can be changed in order to evaluate the functions
*/
#define CHANNEL A2
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 22000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

enum StateType {IDLING, SEARCHING, FORWARD, FINISHED};

arduinoFFT FFT = arduinoFFT();
int target;
int tarIndex;
double maxMag;
float maxTime;
int threshold;
bool magRiseFound;
StateType state;
volatile bool started;
elapsedMillis searchTime;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(START_BUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_BUT_PIN), startButtonISR, FALLING);
  digitalWrite(LED_PIN, LOW);
  target = F1;
  tarIndex = freqToIndex(target);
  threshold = 9000;  // Set arbitrarily. Will need to be changed based on testing. Setting high to test only 1 beacon.
  //  f1done = false;
  //  f2done = false;
  //  f3done = false;
  //  searching = true;
  state = IDLING;
  started = false;
  maxMag = 0;
  maxTime = 0;
  magRiseFound = false;

  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  analogWriteFrequency(PWM_LEFT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteFrequency(PWM_RIGHT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteResolution(16);          // (# of bits)

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
}

void loop() {
  // FFT sampling
  double tarMag = 0;

  int validSamples = 0;  // Counting a sample as valid if the target buzzer is on. This is my idea for dealing with 25% buzzer on time.
  while (validSamples < 10) {
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

    if (vReal[tarIndex] > 500) {  // 500 chosen arbitrarily. Needs testing.
      validSamples++;
      tarMag += vReal[tarIndex - 1] + vReal[tarIndex] + vReal[tarIndex + 1];
    }
  }
  // Take an average of 10 samples for the target frequency
  //  for (int i = 0; i < 10; ++i) {  // Will need to be changed to account for buzzers only producing sound 1/4 of the time
  //    while (!target->available());
  //    tarMag += target->read();
  //  }
  tarMag = tarMag / 10;
  
  int taridx = freqToIndex(F1);
  double test = (vReal[taridx - 1] + vReal[taridx] + vReal[taridx + 1]);

  Serial.print("Tar Mag = ");
  Serial.println(test);
  Serial.print("State = ");
  Serial.println(state);
  
//  for (int i = 1000; i < 3000; i += 1000) {
//    int idx = freqToIndex(i);
//    Serial.print("F = ");
//    Serial.println(i);
//    Serial.print("Index = ");
//    Serial.println(idx);
//    for (int j = idx - 2; j <= idx + 2; j++) {
//      Serial.print("[");
//      Serial.print(j);
//      Serial.print("] = ");
//      Serial.print(vReal[j]);
//      Serial.print("    ");
//    }
//    Serial.println();
//    Serial.println();
//  }

  //  Serial.println(tarMag, 3);
  //  Serial.print("state = ");
  //  Serial.println(state);
  //  Serial.println();

  switch (state) {
    case IDLING:
      if (started) {
        state = SEARCHING;
        maxMag = 0;
        searchTime = 0;
      }
      break;
    case FORWARD:  // Will need to be rewritten to account for buzzers only producing sound 1/4 of the time
      analogWrite(PWM_LEFT_PIN, 6553);
      analogWrite(PWM_RIGHT_PIN, 3276);

      // Check for new target
      if (vReal[freqToIndex(F10)] > threshold && target < F10) {
        target = F10;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F9)] > threshold && target < F9) {
        target = F9;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F8)] > threshold && target < F8) {
        target = F8;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F7)] > threshold && target < F7) {
        target = F7;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F6)] > threshold && target < F6) {
        target = F6;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F5)] > threshold && target < F5) {
        target = F5;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F4)] > threshold && target < F4) {
        target = F4;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F3)] > threshold && target < F3) {
        target = F3;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else if (vReal[freqToIndex(F2)] > threshold && target < F2) {
        target = F2;
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      } else {
        state = FORWARD;
      }

      // Check if car is moving away from beacon
      if (tarMag > maxMag) {
        maxMag = tarMag;
      } else if (tarMag < maxMag * 0.7) {  // 0.7 chosen as an arbitrary threshold. Should be tuned.
        state = SEARCHING;
        searchTime = 0;
        maxMag = 0;
      }
      break;

    // May need to modify to account for buzzers only producing sound 1/4 of the time
    case SEARCHING:  // Aim car at target beacon if searching
      analogWrite(PWM_LEFT_PIN, 4390);  // Turn right
      analogWrite(PWM_RIGHT_PIN, 4390);
      if (tarMag > 500) {  // Don't do anything if target magnitude is insignificant, buzzer probably off
        if (searchTime < 7000) {
          if (tarMag > (maxMag * 1.1)) {  // max * 1.2 is to account for FFT output fluctuation. Needs to be tuned/replaced.
            //          if (maxMag != 0) {
            //            magRiseFound = true;
            //          }
            maxMag = tarMag;
            //          maxTime = micros();
            //        } else if (magRiseFound && tarMag < (maxMag * 0.8)) {  // Turned past beacon. max * 0.8 is to account for FFT output fluctuation. Needs to be tuned/replaced.
            //          turnLeft(micros() - maxTime + 250000);  // 250000 added because it's not turning back far enough. Could also increase speed of left turn.
            //          maxTime = 0;
            //          magRiseFound = false;
            ////          maxMag = 0;
            //          state = FINISHED;  // Setting to FINISHED for search testing. Will want to set to FORWARD in final design.
          } /*else if (micros() - maxTime > 7000000) {  // This assumes time to turn a full circle is 5 seconds. Adjust if necessary. Should be set to some value greater than time to make a full circle.
            maxTime = micros();
            magRiseFound = false;
            maxMag = tarMag;
          } */
        } else if (tarMag >= maxMag * 0.9) {
          state = FINISHED;
        }
      }
      break;

    case FINISHED:
      // Stop
      analogWrite(PWM_LEFT_PIN, 5000);
      analogWrite(PWM_RIGHT_PIN, 5000);
      // Light an LED
      digitalWrite(LED_PIN, HIGH);
      while (1);  // Wait forever
      break;

    default:
      // If somehow not in any state,
      state = SEARCHING;
      searchTime = 0;
      maxMag = 0;
      //        searchTime = 0;
      break;
  }

  delay(20);
}

void goForward(int leftSpeed, int rightSpeed) {
  analogWrite(PWM_LEFT_PIN, leftSpeed);           //6553
  analogWrite(PWM_RIGHT_PIN, rightSpeed);          // 3276
  //  delay(15);
}

void turnLeft(float time) {
  analogWrite(PWM_LEFT_PIN, 5420);
  analogWrite(PWM_RIGHT_PIN, 5420);
  delayMicroseconds(time);
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);
}

void startButtonISR() {
  started = true;
}

int freqToIndex(int f) {
  return round((f * samples) / samplingFrequency);
}
