/* Full control algorithm.
   Current issues:
      Wheels generate significant noise, especially in the 14kHz range.
        May need to alter the searching algorithm to stop when taking readings
        rather than move continuously.
*/

#include <arduinoFFT.h>

#define F1 5000
#define F2 6000
#define F3 7000
#define F4 8000
#define F5 9000
#define F6 10000
#define F7 11000
#define F8 12000
#define F9 13000
#define F10 14000
//#define F1 1000
//#define F2 2000
//#define F3 3000
//#define F4 4000
//#define F5 5000
//#define F6 6000
//#define F7 7000
//#define F8 8000
//#define F9 9000
//#define F10 10000

#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
#define LED_PIN       13
#define START_BUT_PIN 7
#define CALIBRATION_PIN A9
#define CALIBRATION_SWITCH_PIN 11
#define STRAIGHT_SPEED_LEFT 4440  // 4340
#define STRAIGHT_BASE_RIGHT 5310  // 5410
#define RIGHT_TURN_SPEED 4490  // 4490
#define LEFT_TURN_SPEED_BASE 5320

/*
  These values can be changed in order to evaluate the functions
*/
#define CHANNEL A2
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 30000; //Hz
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
const int threshold[10] = {180, 120, 90, 90, 90, 90, 75, 75, 70, 70};  // Needs adjustment
bool magRiseFound;
volatile StateType state;
volatile bool started;
double lastButtonPress = 0;

uint16_t calibrateRead = 0;
uint16_t straightSpeedRight = STRAIGHT_BASE_RIGHT;
uint16_t leftTurnSpeed = LEFT_TURN_SPEED_BASE;

elapsedMillis forwardTime = 0;

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

  switch (state) {
    case IDLING:
      //      while (!started);
      if (started)  
        toSearching(target);
      //        goForward();  // testing the noise of the wheels going forward
      break;
    case FORWARD:  // Will need to be rewritten to account for buzzers only producing sound 1/4 of the time
//      goForward();

      // Periodically check for new target
      if (forwardTime >= 2000) {
        // Wheels generate significant noise, especially in the 14kHz range.
        // So, we must stop before looking for new signals.
        stopCar();
        delay(500);  // Wait a bit just to ensure the wheels are stopped before sampling
        forwardTime = 0;
        // Sample for 500ms while stopped
        while (forwardTime < 500) {
          fftSample();
        }
        if (magnitudes[9] > threshold[9] && target < F10) {
          toSearching(F10);
          break;
        } else if (magnitudes[8] > threshold[8] && target < F9) {
          toSearching(F9);
          break;
        } else if (magnitudes[7] > threshold[7] && target < F8) {
          toSearching(F8);
          break;
        } else if (magnitudes[6] > threshold[6] && target < F7) {
          toSearching(F7);
          break;
        } else if (magnitudes[5] > threshold[5] && target < F6) {
          toSearching(F6);
          break;
        } else if (magnitudes[4] > threshold[4] && target < F5) {
          toSearching(F5);
          break;
        } else if (magnitudes[3] > threshold[3] && target < F4) {
          toSearching(F4);
          break;
        } else if (magnitudes[2] > threshold[2] && target < F3) {
          toSearching(F3);
          break;
        } else if (magnitudes[1] > threshold[1] && target < F2) {
          toSearching(F2);
          break;
        } /*else {
        state = FORWARD;
      }*/

        //       Check if car is moving away from beacon
        if (tarMag > maxMag) {
          maxMag = tarMag;
        } else if (tarMag < maxMag * 0.5) {  // 0.5 chosen as an arbitrary threshold. Should be tuned.
          Serial.println("GONE SEARCHING");  // Test code to see if this event has occurred
          delay(2000);
          toSearching(target);
          break;
        }

        forwardTime = 0;
//        goForward();
      }


      // Draft of code for using distance sensor
      /*
         distance = distanceFunc();  // If there is a function to call, do so
         if (distance < 10) {
           if (target == F10 && tarMag > 500) {  // Need a real value. 500 tentative. If approaching last beacon, keep going a bit, then be done!
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
             toSearching(target);
           }
         }
      */

      break;

    case SEARCHING:  // Aim car at target beacon if searching
      // I don't think we need this first "if" anymore because the fft sampling handles it. Should test without it.
      //      if (tarMag > threshold[(target / 1000) - (F1 / 1000)]) {  // Don't do anything if target magnitude is insignificant, buzzer probably off
      if (tarMag > (maxMag * 1.1)) {  // max * 1.1 is to account for FFT output fluctuation. Needs to be tuned/replaced.
        if (maxMag != 0) {
          magRiseFound = true;
        }
        maxMag = tarMag;
        //          Serial.print("maxMag = ");
        //          Serial.println(maxMag);
        maxTime = micros();
      } else if (magRiseFound && tarMag < (maxMag * 0.9)) {  // Turned past beacon. max * 0.9 is to account for FFT output fluctuation. Needs to be tuned/replaced.
        turnLeft(micros() - maxTime);  // May need to add an offset if moving average filter is too slow
        maxTime = 0;
        magRiseFound = false;
        maxMag = tarMag;  // For accurate detection of moving away from beacon in FORWARD state.
        state = FORWARD;  // Setting to FINISHED for search testing. Will want to set to FORWARD in final design.
      } else if (micros() - maxTime > 7000000) {  // This assumes time to turn a full circle is 5 seconds. Adjust if necessary. Should be set to some value greater than time to make a full circle.
        maxTime = micros();
        magRiseFound = false;
        maxMag = tarMag;
      }
      //      }
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
      toSearching(target);
      //        searchTime = 0;
      break;
  }

  fftSample();

  /* Test code for displaying raw FFT data */
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



  //  Serial.println();
  //  Serial.print("state = ");
  //  Serial.println(state);
  //  Serial.println();

  delay(40);
}

void startButtonISR() {
  if ((millis() - lastButtonPress) > 100) {  // Prevent button bounce
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

void toSearching(int tar) {
  stopCar();
  target = tar;
  tarFFTindex = freqToIndex(tar);

  // Reset running average for new target
  tarAvgSum = 0;
  tarAvgPos = 0;
  tarMag = 0;
  for (int i = 0; i < AVG_NUMBER; ++i) {
    tarAvgHistory[i] = 0;
  }

  // Sample for 1 second to populate moving average with current readings so there is no faulty rise detected
  double startTime = millis();
  while (millis() < (startTime + 1000)) {
    fftSample();
  }
  turnRight();
  maxMag = 0;
  //  Serial.println("In transition");
  state = SEARCHING;
}

