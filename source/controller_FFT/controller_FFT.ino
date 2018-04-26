/* This code is a draft of a full control algorithm.
    The code to find the direction of a beacon is all that is
    implemented so far, but that seems to be working decently.
    There are still a couple bugs:
      The search sometimes finishes in the wrong direction, not
        sure why yet.
*/

#include <arduinoFFT.h>

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

#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
#define LED_PIN       13
#define START_BUT_PIN 7
#define CALIBRATION_PIN A9
#define CALIBRATION_SWITCH_PIN 11
#define STRAIGHT_SPEED_LEFT 4340
#define STRAIGHT_BASE_RIGHT 5410
#define RIGHT_TURN_SPEED 4490
#define LEFT_TURN_SPEED_BASE 5320

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
#define AVG_NUMBER 20
double avgHistory[10][AVG_NUMBER] = {0};
double avgSum[10] = {0};
int avgPos[10] = {0};
double magnitudes[10] = {0};
double tarAvgSum = 0;
double tarAvgHistory[AVG_NUMBER] = {0};
int tarAvgPos = 0;
double tarMag = 0;

enum StateType {IDLING, SEARCHING, FORWARD, FINISHED, CALIBRATION_STRAIGHT, CALIBRATION_TURN};

arduinoFFT FFT = arduinoFFT();
int target;
int tarFFTindex;
double maxMag;
float maxTime;
const int threshold[10] = {400, 400, 300, 300, 200, 200, 200, 100, 500, 40};  // May need adjustment
bool magRiseFound;
StateType state;
volatile bool started;
double lastButtonPress = 0;

uint16_t calibrateRead = 0;
uint16_t straightSpeedRight = STRAIGHT_BASE_RIGHT;
uint16_t leftTurnSpeed = LEFT_TURN_SPEED_BASE;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(START_BUT_PIN, INPUT_PULLUP);
  pinMode(CALIBRATION_PIN, INPUT);
  pinMode(CALIBRATION_SWITCH_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(START_BUT_PIN), startButtonISR, FALLING);
  digitalWrite(LED_PIN, LOW);
  target = F1;
  tarFFTindex = freqToIndex(target);
  //  threshold = 1200;  // Set arbitrarily. Will need to be changed based on testing. Setting high to test only 1 beacon.
  //  threshold[0] = 400;
  //  f1done = false;
  //  f2done = false;
  //  f3done = false;
  //  searching = true;
  if (digitalRead(CALIBRATION_SWITCH_PIN) == 0) {
    state = IDLING;
  } else {
    state = CALIBRATION_STRAIGHT;
  }
  started = false;
  maxMag = 0;
  maxTime = 0;
  magRiseFound = false;
  tarMag = 0;

  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  analogWriteFrequency(PWM_LEFT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteFrequency(PWM_RIGHT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteResolution(16);          // (# of bits)

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
}

void loop() {
  //  Serial.print("state = ");
  //  Serial.println(state);
  Serial.print("Target = ");
  Serial.println(target);
  Serial.print("magnitudes[9] = ");
  Serial.println(magnitudes[9]);

  switch (state) {
    case IDLING:
      while (!started);
      //      if (started) {
      toSearching();
      //      }
      break;
    case FORWARD:  // Will need to be rewritten to account for buzzers only producing sound 1/4 of the time
      analogWrite(PWM_LEFT_PIN, STRAIGHT_SPEED_LEFT);
      analogWrite(PWM_RIGHT_PIN, straightSpeedRight);

      // Check for new target
      if (magnitudes[9] > threshold[9] && target < F10) {
        target = F10;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[8] > threshold[8] && target < F9) {
        target = F9;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[7] > threshold[7] && target < F8) {
        target = F8;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[6] > threshold[6] && target < F7) {
        target = F7;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[5] > threshold[5] && target < F6) {
        target = F6;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[4] > threshold[4] && target < F5) {
        target = F5;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[3] > threshold[3] && target < F4) {
        target = F4;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[2] > threshold[2] && target < F3) {
        target = F3;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else if (magnitudes[1] > threshold[1] && target < F2) {
        target = F2;
        tarFFTindex = freqToIndex(target);
        toSearching();
        break;
      } else {
        state = FORWARD;
      }

      // Check if car is moving away from beacon
      //      if (tarMag > maxMag) {
      //        maxMag = tarMag;
      //      } else if (tarMag < maxMag * 0.7) {  // 0.7 chosen as an arbitrary threshold. Should be tuned.
      //        toSearching();
      //      }

      // Draft of code for using distance sensor
      /*
         distance = distanceFunc();
         if (distance < 10) {
           if (target == F10) {  // If approaching last beacon, keep going a bit, then be done!
             delay(2000);  // Tune delay to time it takes to move forward 10cm
             state = FINISHED;
             break;
           } else {  // Approaching another beacon. Stop and then... do something
             stopCar();
             delay(500);  // might need small delay for movement change
             turnRight();
             while (distance < 15) {
               // Call distance check function if necessary
             }
             stopCar();
             delay(500);  // might need small delay for movement change
             goForward();
             delay(2000);  // 2 seconds chosen arbitrarily. Test and tune.
             stopCar();
             delay(500);  // might need small delay for movement change
             toSearching();
           }
         }
      */

      break;

    // May need to modify to account for buzzers only producing sound 1/4 of the time
    case SEARCHING:  // Aim car at target beacon if searching
      if (tarMag > 900) {  // Don't do anything if target magnitude is insignificant, buzzer probably off
        if (tarMag > (maxMag * 1.1)) {  // max * 1.1 is to account for FFT output fluctuation. Needs to be tuned/replaced.
          if (maxMag != 0) {
            magRiseFound = true;
          }
          maxMag = tarMag;
          //          Serial.print("maxMag = ");
          //          Serial.println(maxMag);
          maxTime = micros();
        } else if (magRiseFound && tarMag < (maxMag * 0.8)) {  // Turned past beacon. max * 0.8 is to account for FFT output fluctuation. Needs to be tuned/replaced.
          turnLeft(micros() - maxTime + 0);  // 150000 added because it's not turning back far enough. Could also increase speed of left turn.
          maxTime = 0;
          magRiseFound = false;
          //          maxMag = 0;
          state = FORWARD;  // Setting to FINISHED for search testing. Will want to set to FORWARD in final design.
        } else if (micros() - maxTime > 7000000) {  // This assumes time to turn a full circle is 5 seconds. Adjust if necessary. Should be set to some value greater than time to make a full circle.
          maxTime = micros();
          magRiseFound = false;
          maxMag = tarMag;
          //          state = FINISHED;  // Setting to FINISHED for search testing. Will want to set to FORWARD in final design.
        }
      }
      break;

    case FINISHED:
      stopCar();
      // Light an LED
      digitalWrite(LED_PIN, HIGH);
      while (1);  // Wait forever
      break;

    case CALIBRATION_STRAIGHT:
      calibrate_straight();
      break;

    case CALIBRATION_TURN:
      calibrate_turn();
      break;

    default:
      // If somehow not in any state,
      toSearching();
      //        searchTime = 0;
      break;
  }

  fftSample();

  //  Serial.print("tarMag before division = ");
  //  Serial.println(tarMag, 3);
  //  Serial.print("valid samples = ");
  //  Serial.println(validSamples);
  //  Serial.print("target value in array = ");
  //  Serial.println(vReal[tarFFTindex]);
  //  tarMag = tarMag / validSamples;


  //      for (int i = 1000; i < 3000; i += 1000) {
  //        int idx = freqToIndex(i);
  //        Serial.print("F = ");
  //        Serial.println(i);
  //        Serial.print("Index = ");
  //        Serial.println(idx);
  //        for (int j = idx - 2; j <= idx + 2; j++) {
  //          Serial.print("[");
  //          Serial.print(j);
  //          Serial.print("] = ");
  //          Serial.print(vReal[j]);
  //          Serial.print("    ");
  //        }
  //        Serial.println();
  //        Serial.println();
  //      }
  //

  //  for (int i = 0; i < 10; ++i) {
  //    Serial.print(magnitudes[i], 3);
  //    Serial.print("    ");
  //  }
  //  Serial.println();
  //  Serial.print("state = ");
  //  Serial.println(state);
  //  Serial.println();
  //
  //  static int timecount = 0;
  //  static double times[100] = {0};
  //  if (timecount < 100) {
  //    times[timecount] = millis();
  //    timecount++;
  //  } else {
  //    for (int i = 0; i < 100; ++i) {
  //      Serial.println(times[i]);
  //      times[i] = 0;
  //    }
  //    timecount = 0;
  //  }
  
  delay(40);
}

void startButtonISR() {
  if ((millis() - lastButtonPress) > 50) {  // Prevent button bounce
    lastButtonPress = millis();
    if (state == IDLING) {
      started = true;
    } else if (state == CALIBRATION_STRAIGHT) {
      state = CALIBRATION_TURN;
    } else if (state == CALIBRATION_TURN) {
      state = IDLING;
    }
  }
}

void toSearching(void) {
  stopCar();
  double startTime = millis();
  // Sample for 1 second to populate moving average with current readings so there is no faulty rise detected
  while (millis() < (startTime + 1000)) {
    fftSample();
  }
  turnRight();
  maxMag = 0;
  //  Serial.println("In transition");
  state = SEARCHING;
}


