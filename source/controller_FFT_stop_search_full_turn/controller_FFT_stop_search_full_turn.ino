/* =====   Real-time Embedded Systems -- Final Project, Group 12   =====
 *  Anthony Fisher
 *  Rui Zhang
 *  Nikhil Shinde
 *  Keonghwan Oh
 *  
 *  =====   Brief Algorithm Description   =====
 *  The car turns in a full circle, stopping to sample
 *  data after short turns. After the full circle, the car continues to turn
 *  until it finds another value comparable to the recorded maximum.
 *  
 *  While moving forwards, the car stops periodically to sample audio data.
 *  If the value has fallen too low from the recorded maximum, the car re-orients
 *  to the proper direction. To do this, it performs a search with left turns.
 *  
 *  At any point in searching or moving forwards, new frequencies are also 
 *  detected. If a new (higher) frequency is detected above a given threshold, 
 *  it is set as the target and the process repeats.
 *  
 *  If the car ever gets completely lost and can't hear it's target frequency at all
 *  during a full-circle search, it will reset to the first target in the sequence
 *  and search for anything it can hear.
*/

#include <arduinoFFT.h>

/* Frequencies to search for. Three different versions for 
 *  the three different specification changes we went through.
 */
//#define LOW_FQS
#define MID_FQS
//#define HIGH_FQS
#ifdef MID_FQS
#define F1 5000
#define F2 5500
#define F3 6000
#define F4 6500
#define F5 7000
#define F6 7500
#define F7 8000
#define F8 8500
#define F9 9000
#define F10 9500
#elif defined(HIGH_FQS)
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
#elif defined(LOW_FQS)
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
#endif

#define RIGHT 0
#define LEFT  1

// Teensy pins used
#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
#define LED_PIN       13
#define START_BUT_PIN 7
#define CALIBRATION_PIN A9
#define CALIBRATION_SWITCH_PIN 11
#define TRIG_PIN 8
#define ECHO_PIN 9

// Settings for wheel speeds with and without fixed front wheel.
//#define FAST_SPEEDS
//#define FIXED_WHEEL

//#ifdef FAST_SPEEDS
#define STRAIGHT_SPEED_LEFT 4140 //4240
#define STRAIGHT_BASE_RIGHT 5593 //5493
#define BASE_FORWARD_MICROS 1300000 //1500000

#ifdef FIXED_WHEEL
#define RIGHT_TURN_SPEED 4000
#define LEFT_TURN_SPEED_BASE 5748
#define BASE_TURN_MICROS 210000
#define NUM_SEARCH_STOPS 20
#define NUM_SEARCH_STOPS_MAX 45
#else
#define RIGHT_TURN_SPEED 4290
#define LEFT_TURN_SPEED_BASE 5458
#define BASE_TURN_MICROS 264000
#define NUM_SEARCH_STOPS 20
#define NUM_SEARCH_STOPS_MAX 44
#endif

//#else
//#define STRAIGHT_SPEED_LEFT 4440  // 4440
//#define STRAIGHT_BASE_RIGHT 5280  // 5404
//#define RIGHT_TURN_SPEED 4490  // 4490
//#define LEFT_TURN_SPEED_BASE 5325
//#define BASE_FORWARD_MICROS 4000000
//#define BASE_TURN_MICROS 600000
//#endif

/*
  FFT settings and variables.
*/
#define CHANNEL A2
#define LISTEN_TIME_MILLIS 1250
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 20000; //Hz
unsigned long microseconds;
unsigned int sampling_period_us;

/*
  These are the FFT input and output vectors.
  Input vectors receive computed results from FFT.
  Also the running average 
*/
double vReal[samples];
double vImag[samples];
#define AVG_NUMBER 15
double avgHistory[10][AVG_NUMBER] = {0};
double avgSum[10] = {0};
int avgPos[10] = {0};
double magnitudes[10] = {0};
//double tarAvgSum = 0;
//double tarAvgHistory[AVG_NUMBER] = {0};
//int tarAvgPos = 0;
double tarMag = 0;
double maxSample = 0;
uint8_t searchDirection = RIGHT;
bool gotLost = false;

enum StateType {IDLING, SEARCHING, FORWARD, FINISHED, CALIBRATION_STRAIGHT, CALIBRATION_TURN};

arduinoFFT FFT = arduinoFFT();
int target;
int tarFFTindex;
double maxMag;
//float maxTime;
const int targetThreshold[10] =    {30, 30, 30, 30, 30, 30, 30, 30, 40, 40};  // Needs adjustment
const int initialThreshold[10] = {100, 100, 100, 100, 100, 100, 100, 70, 60, 40};  // Needs adjustment
bool magRiseFound;
volatile StateType state;
volatile bool started;
double lastButtonPress = 0;

uint16_t calibrateRead = 0;
uint16_t straightSpeedRight = STRAIGHT_BASE_RIGHT;
uint16_t leftTurnSpeed = LEFT_TURN_SPEED_BASE;

elapsedMillis forwardTime = 0;
elapsedMicros searchTime = 0;
unsigned long turnTime = 0;
unsigned int distance = 400;

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
  magRiseFound = false;
  tarMag = 0;

  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  analogWriteFrequency(PWM_LEFT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteFrequency(PWM_RIGHT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteResolution(16);          // (# of bits)

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
}

void loop() {

  switch (state) {
    case IDLING:
      if (started)
        toSearching(target);
      fftSample();  // For testing. Could remove for final.
//                  distance = ultraSonic();  // For testing. Remove for final.
      //            goForward();  // testing the noise of the wheels going forward
      break;
    case FORWARD: 
      goForward();

      // Periodically check for new target
      if (forwardTime > BASE_FORWARD_MICROS / 1000) {
        // Wheels generate significant noise.
        // So, we must stop before looking for new signals.
        stopCar();
        delay(100);  // Wait a bit just to ensure the wheels are stopped before sampling
        forwardTime = 0;
        
        // Sample while stopped. Record maximum sampled target value to use as new target magnitude.
        Serial.println();
        maxSample = 0;
        while (forwardTime < LISTEN_TIME_MILLIS) {
          fftSample();
          if (magnitudes[(target - F1) / (F2 - F1)] > maxSample) {
            maxSample = magnitudes[(target - F1) / (F2 - F1)];
          }
        }
        tarMag = maxSample;
        Serial.println();
        Serial.print("tarMag forward = ");
        Serial.println(tarMag);

        // Check to see if any new beacon was detected during sampling.
        if (newFreqCheck()) {
          /* For testing random long stop when going forward */
          stopCar();
          digitalWrite(LED_PIN, HIGH);
          delay(3000);
          digitalWrite(LED_PIN, LOW);
          turnRight();
          /* End test section */
          break;
        }

        //  Check if car is moving away from beacon. If so, re-orient.
        if (tarMag > maxMag) {
          maxMag = tarMag;
        } else if (tarMag < maxMag * 0.7 || tarMag < targetThreshold[(target - F1) / (F2 - F1)]) {  
          //          Serial.println("GONE SEARCHING");  // Test code to see if this event has occurred
          //          delay(2000);
          toSearching();
          break;
        }
        
        forwardTime = 0;
        goForward();
      }

      // While moving forward, check to make sure car isn't too close to something.
      distance = ultraSonic();  
      if (distance > 0 && distance < 20) {
        /* If the car is approaching something, but the last beacon is the current 
         *  target and that beacon's magnitude is larger than all others, then proceed
         *  forward enough to hit the beacon and move to the FINISHED state.
         */
        if (target == F10) {  
          bool lastIsLoud = true;
          for (int i = 0; i < 9; ++i) {
            if (tarMag < magnitudes[i]) {
              lastIsLoud = false;
              break;
            }
          }
          if (lastIsLoud) {  // If approaching last beacon, keep going a bit, then be done!
            delay(2000);  
            state = FINISHED;
            break;
          }
        } else {  // Approaching another beacon. Stop, turn right a bit, go forward, then search for the target direction again.
          evasiveManeuvers();
        }
      }

      break;

    case SEARCHING:  // Aim car at target beacon if searching
      if (searchTime > BASE_TURN_MICROS) {  // Turn and then stop periodically to sample so wheel noise doesn't interfere
        turnTime += searchTime;
        stopCar();
        delay(100);  // Wait a bit just to ensure the wheels are stopped before sampling
        searchTime = 0;
        
        // Sample while stopped. Record maximum sampled target value to use as new target magnitude.
        Serial.println();
        maxSample = 0;
        while (searchTime < LISTEN_TIME_MILLIS * 1000) {
          fftSample();
          if (magnitudes[(target - F1) / (F2 - F1)] > maxSample) {
            maxSample = magnitudes[(target - F1) / (F2 - F1)];
          }
        }
        tarMag = maxSample;
        Serial.println();
        Serial.print("tarMag searching = ");
        Serial.println(tarMag);

        // Check to see if any new beacon was detected during sampling.
        if (newFreqCheck()) {
          /* For testing random long stop when going forward */
          stopCar();
          digitalWrite(LED_PIN, HIGH);
          delay(1000);
          digitalWrite(LED_PIN, LOW);
          turnRight();
          /* End test section */
          break;
        }

        // Update maximum recorded magnitude.
        if (tarMag > (maxMag * 1.1)) {  // max * 1.1 is to account for FFT output fluctuation. 
          maxMag = tarMag;
          Serial.println();
          Serial.print("MAXMAG = ");
          Serial.println(maxMag);
          Serial.println();
        }

        /* If the car has made a full circle or previously gotten lost, and the target magnitude is 
         *  comparable to the maximum recorded magnitude, and that maximum is above the threshold,
         *  then go FORWARD.
         */
        if ((turnTime > BASE_TURN_MICROS * NUM_SEARCH_STOPS || gotLost) && 
            (tarMag > maxMag * 0.75) && 
            (maxMag > targetThreshold[(target - F1) / (F2 - F1)])) {  
          state = FORWARD;  
          maxMag = tarMag;
          forwardTime = 0;
          break;
        }

        // If searching for a very long time, start the search over.
        if (turnTime > BASE_TURN_MICROS * NUM_SEARCH_STOPS_MAX) {
          if (maxMag < initialThreshold[(target - F1) / (F2 - F1)]) {  // completely lost target frequency. Check for any frequency.
            target = F1;
            if (newFreqCheck()) {
              toSearching(target);
              break;
            }
          } else {  // If target is detectable, but the max was never found again, reset max and try again.
            toSearching(target);
            break;
          }
        }

        searchTime = 0;
        if (searchDirection == RIGHT) {
          turnRight();
        } else {
          turnLeft();
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
      /* Calibrate wheels to move straight, even though it doesn't really 
       *  work due to the front caster wheel being really crappy.
       */
      calibrate_straight();
      break;

    case CALIBRATION_TURN:
      calibrate_turn();
      break;

    default:
      // If somehow not in any state,
      toSearching(target);
      break;
  }

  delay(40);
}

/* Start when the start button is pressed. Starting
 *  state depends on the switch value.
 */
void startButtonISR() {
  if ((millis() - lastButtonPress) > 300) {  // Prevent button bounce
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

/* This version of toSearching is called when a new beacon has
    been detected and its direction must be determined.
*/
void toSearching(int tar) {
  stopCar();
  target = tar;
  tarFFTindex = freqToIndex(tar);

  // DEPRECATED: Reset running average for new target
//  tarAvgSum = 0;
//  tarAvgPos = 0;
  tarMag = 0;
//  for (int i = 0; i < AVG_NUMBER; ++i) {
//    tarAvgHistory[i] = 0;
//  }

  // Sample for 1 second to populate moving average with current readings.
  double startTime = millis();
  while (millis() < (startTime + 1000)) {
    fftSample();
  }
  maxMag = 0;
  state = SEARCHING;
  gotLost = false;
  searchDirection = RIGHT;  // Turn right for normal search.
  turnRight();
  searchTime = 0;
  turnTime = 0;
}

/* This version of toSearching gets called if the car loses a beacon
    it is tracking while moving forwards. After a search, the front wheel
    doesn't fully return to a straight position, making the car veer in the
    direction of the search turn rather than going straight. This function
    will search to the left, since the car usually goes right instead of 
    straight forward.
*/
void toSearching(void) {
  stopCar();

  // Sample for 1 second to populate moving average with current readings.
  double startTime = millis();
  while (millis() < (startTime + 1000)) {
    fftSample();
  }
  state = SEARCHING;
  gotLost = true;  // Indicates that target has previously been found, but getting off course.
  searchDirection = LEFT;
  turnLeft();
  searchTime = 0;
  turnTime = 0;
}

/* Checks to see if any frequency further in the sequence is above
 *  the minimum threshold. If so, sets the target to that frequency
 *  and starts a search to find where the beacon is.
 *  Returns true if a new frequency was found and false if not.
 */
bool newFreqCheck(void) {
  bool retVal = false;
  if (magnitudes[9] > initialThreshold[9] && target < F10) {
    toSearching(F10);
    retVal = true;
  } else if (magnitudes[8] > initialThreshold[8] && target < F9) {
    toSearching(F9);
    retVal = true;
  } else if (magnitudes[7] > initialThreshold[7] && target < F8) {
    toSearching(F8);
    retVal = true;
  } else if (magnitudes[6] > initialThreshold[6] && target < F7) {
    toSearching(F7);
    retVal = true;
  } else if (magnitudes[5] > initialThreshold[5] && target < F6) {
    toSearching(F6);
    retVal = true;
  } else if (magnitudes[4] > initialThreshold[4] && target < F5) {
    toSearching(F5);
    retVal = true;
  } else if (magnitudes[3] > initialThreshold[3] && target < F4) {
    toSearching(F4);
    retVal = true;
  } else if (magnitudes[2] > initialThreshold[2] && target < F3) {
    toSearching(F3);
    retVal = true;
  } else if (magnitudes[1] > initialThreshold[1] && target < F2) {
    toSearching(F2);
    retVal = true;
  }
  return retVal;
}


