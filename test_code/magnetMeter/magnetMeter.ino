#include <math.h>

#include <Wire.h> //I2C Arduino Library
#define addr 0x0D //I2C Address for The HMC5883
#define PWM_LEFT_PIN  3
#define PWM_RIGHT_PIN 5
#define LED_PIN       13
#define START_BUT_PIN 7
#define CALIBRATION_PIN A9
#define CALIBRATION_SWITCH_PIN 11
#define TRIG_PIN 8
#define ECHO_PIN 9

#define FAST_SPEEDS
#ifdef FAST_SPEEDS
#define STRAIGHT_SPEED_LEFT 4240
#define STRAIGHT_BASE_RIGHT 5493
#define RIGHT_TURN_SPEED 4290
#define LEFT_TURN_SPEED_BASE 5458
#define BASE_FORWARD_MICROS 1500000
#define BASE_TURN_MICROS 264000
#else
#define STRAIGHT_SPEED_LEFT 4440  // 4440
#define STRAIGHT_BASE_RIGHT 5280  // 5404
#define RIGHT_TURN_SPEED 4490  // 4490
#define LEFT_TURN_SPEED_BASE 5325
#define BASE_FORWARD_MICROS 4000000
#define BASE_TURN_MICROS 600000
#endif

float prevSlope = 0.0;
enum StateType {IDLING, FORWARD, FINISHED, CALIBRATION_STRAIGHT, CALIBRATION_TURN};

volatile StateType state;
volatile bool started;
double lastButtonPress = 0;

uint16_t calibrateRead = 0;
uint16_t straightSpeedRight = STRAIGHT_BASE_RIGHT;
uint16_t leftTurnSpeed = LEFT_TURN_SPEED_BASE;

void setup() {

  Serial.begin(9600);
  Wire.begin();


  Wire.beginTransmission(addr); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // Set the Register
  Wire.endTransmission();

  pinMode(LED_PIN, OUTPUT);
  pinMode(START_BUT_PIN, INPUT_PULLUP);
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(START_BUT_PIN), startButtonISR, FALLING);
  
  analogWriteFrequency(PWM_LEFT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteFrequency(PWM_RIGHT_PIN, 50);  // (pin, frequency in Hz)
  analogWriteResolution(16);          // (# of bits)

  pinMode(CALIBRATION_PIN, INPUT);
  pinMode(CALIBRATION_SWITCH_PIN, INPUT);

  started = false;
  if (digitalRead(CALIBRATION_SWITCH_PIN) == 0) {
    state = IDLING;
  } else {
    state = CALIBRATION_STRAIGHT;
  }


  
}

void loop() {
  
//  switch(state) {
//    case IDLING: 
//      if (started) {
//        delay(3000);
//        digitalWrite(LED_PIN, LOW);
//        turnCircle();
//        state = FORWARD;
//      }
//      break;
//      
//    case FORWARD: 
//      float curSlope;
//      curSlope = slopeCheck();
//      if (fabs(curSlope - prevSlope) >= 0.3 && prevSlope > 0.001 ) {
//        state = FINISHED;
//      }
//      prevSlope = curSlope;
//      goForward();
//      break;
//
//    case FINISHED: 
//      stopCar();
//      // Light an LED
//      digitalWrite(LED_PIN, HIGH);
//      while (1);  // Wait forever
//      break;
//
//    case CALIBRATION_STRAIGHT:
//      digitalWrite(LED_PIN, LOW);
//      calibrate_straight();
//      break;
//
//    case CALIBRATION_TURN:
//      digitalWrite(LED_PIN, LOW);
//      calibrate_turn();
//      break;

  //}
  slopeCheck();
      
}

float slopeCheck() {
  int x, y, z; //triple axis data
  float slope;
  //Tell the HMC what regist to begin writing data into


  Wire.beginTransmission(addr);
  Wire.write(0x00); //start with register 3.
  Wire.endTransmission();

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if (6 <= Wire.available()) {
    x = Wire.read(); //MSB  x
    x |= Wire.read() << 8; //LSB  x
    z = Wire.read(); //MSB  z
    z |= Wire.read() << 8; //LSB z
    y = Wire.read(); //MSB y
    y |= Wire.read() << 8; //LSB y
  }

  if (x < 1000) {
    x += 65536;
  }
  if (y < 1000) {
    x += 65536;
  }

  // Show Values
  Serial.print("X Value: ");
  Serial.println(x);
  Serial.print("Y Value: ");
  Serial.println(y);
  Serial.print("Z Value: ");
  Serial.println(z);
  Serial.println();
  
  slope = atan2(x % 10000 , y % 10000);
  Serial.print("x: ");
  Serial.println((x % 10000));
  Serial.print("y: ");
  Serial.println(y % 10000);
  Serial.println(slope);
  delay(50);
  
  return slope;
}

void startButtonISR() {
  if ((millis() - lastButtonPress) > 300) {  // Prevent button bounce
    lastButtonPress = millis();
    if (state == IDLING) {
      started = true;
      digitalWrite(LED_PIN, HIGH);
    } 
  }
}

void turnCircle() {
  turnRight(7000);
}

