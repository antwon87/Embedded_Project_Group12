#include <Audio.h>

#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
#define LED_PIN       13
#define START_BUT_PIN 7

enum StateType {IDLING, SEARCHING, FORWARD, FINISHED};

//IntervalTimer turnTimer;
AudioInputAnalog adc1;
AudioAnalyzeToneDetect *target;
AudioAnalyzeToneDetect f1;
AudioAnalyzeToneDetect f2;
AudioAnalyzeToneDetect f3;
AudioAnalyzeToneDetect f4;
AudioConnection wire1(adc1, f1);
AudioConnection wire2(adc1, f2);
AudioConnection wire3(adc1, f3);
AudioConnection wire4(adc1, f4);
float maxMag;
float maxTime;
int threshold;
bool f1done;
bool f2done;
bool f3done;
//bool searching;
bool magRiseFound;
volatile StateType state;
//elapsedMillis targetingTime;
volatile bool started;

void setup() {
  AudioMemory(12);
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(START_BUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_BUT_PIN), startButtonISR, FALLING);
  digitalWrite(LED_PIN, LOW);
  f1.frequency(1000);
  f2.frequency(2000);
  f3.frequency(3000);
  f4.frequency(4000);
  target = &f1;
  threshold = 0.9;  // Set arbitrarily. Will need to be changed based on testing. Setting high to test only 1 beacon.
  f1done = false;
  f2done = false;
  f3done = false;
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
}

void loop() {
  // FFT sampling

  float f1Mag = f1.read();
  float f2Mag = f2.read();
  float f3Mag = f3.read();
  float f4Mag = f4.read();
  float tarMag = 0;

  // Take an average of 10 samples for the target frequency
  for (int i = 0; i < 10; ++i) {  // Will need to be changed to account for buzzers only producing sound 1/4 of the time
    while (!target->available());
    tarMag += target->read();
  }
  tarMag = tarMag / 10;

  Serial.println(tarMag, 3);
  Serial.print("state = ");
  Serial.println(state);
  Serial.println();

  switch (state) {
    case IDLING:
      if (started) {
        state = SEARCHING;
      }
      break;
    case FORWARD:  // Will need to be rewritten to account for buzzers only producing sound 1/4 of the time
      analogWrite(PWM_LEFT_PIN, 6553);
      analogWrite(PWM_RIGHT_PIN, 3276);

      // Check for new target
      if (f4Mag) {
        target = &f4;
        f1done = true;
        f2done = true;
        f3done = true;
        state = SEARCHING;
        //        maxMag = 0;
        //        continue;
      } else if (f3Mag && !f3done) {
        target = &f3;
        f1done = true;
        f2done = true;
        state = SEARCHING;
        //        maxMag = 0;
        //        continue;
      } else if (f2Mag && !f2done) {
        target = &f2;
        state = SEARCHING;
        //        maxMag = 0;
        //        continue;
        //      f1done = true;
      } else {
        state = FORWARD;
      }
      break;

    // Will need to modify to account for buzzers only producing sound 1/4 of the time
    case SEARCHING:  // Aim car at target beacon if searching
      analogWrite(PWM_LEFT_PIN, 4240);  // Turn right
      analogWrite(PWM_RIGHT_PIN, 4240);
      if (tarMag > 0.04) {  // Don't do anything if target magnitude is insignificant, buzzer probably off
        if (tarMag > (maxMag * 1.2)) {  // max * 1.2 is to account for FFT output fluctuation. Needs to be tuned/replaced.
          if (maxMag != 0) {
            magRiseFound = true;
          }
          maxMag = tarMag;
          maxTime = micros();
        } else if (magRiseFound && tarMag < (maxMag * 0.8)) {  // Turned past beacon. max * 0.8 is to account for FFT output fluctuation. Needs to be tuned/replaced.
          turnLeft(micros() - maxTime);
          maxTime = 0;
          magRiseFound = false;
          maxMag = 0;
          state = FINISHED;  // Setting to FINISHED for search testing. Will want to set to TARGETING or FORWARD in final design.
          //        targetingTime = 0;  // Use if state is changing to TARGETING
        }
      }
      break;

    case FINISHED:
      // Light an LED
      digitalWrite(LED_PIN, HIGH);
      while (state == FINISHED);  // Wait forever
      break;

    default:
      // If somehow not in any state,
      state = SEARCHING;
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
  analogWrite(PWM_LEFT_PIN, 5570);
  analogWrite(PWM_RIGHT_PIN, 5570);
  delayMicroseconds(time);
  analogWrite(PWM_LEFT_PIN, 5000);
  analogWrite(PWM_RIGHT_PIN, 5000);

  //  turnTimer.begin(stopTurnISR, time);
}

//void stopTurnISR() {
//  analogWrite(PWM_LEFT_PIN, 5000);
//  analogWrite(PWM_RIGHT_PIN, 5000);
//  turnTimer.end();
//}

void startButtonISR() {
  started = true;
  if (state == FINISHED) {  // For testing
    state = IDLING;
  }
}

