/*
 
MIT License

Copyright (c) 2022 Crash Works 3D

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

DESCRIPTION
====================
The purpose of this code is to automate the servos, LEDs and pixels for the Predator

ORIGINAL CREATOR
====================
Moe Sizzlac
3D Model: https://www.thingiverse.com/thing:4056653
Moe's Predator Link: http://www.moesizzlac.com/projects-predator.php

DEVELOPED BY
====================
Crash Works 3D: Cranshark, Dropwire
Link Tree: https://linktr.ee/crashworks3d

WORKING DEMO
====================
[TODO]

*/

// Version.  Don't change unless authorized by Cranshark
#define VERSION "0.0.1.1"

// Uncomment to run unit tests
#define RUN_UNIT_TESTS

// Referenced libraries
// Library to make servos work
// See: https://github.com/ArminJo/ServoEasing
#define USE_LEIGHTWEIGHT_SERVO_LIB // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.

#include "ServoEasing.hpp"

// Libraries to make the accelerometer work
// See: https://github.com/rfetick/MPU6050_light
#include "Wire.h"
#include <MPU6050_light.h>

// Library to make sound work
// [TODO: DFPlayer?]

// Library to make WS2812 pixels work
// See: https://github.com/FastLED/FastLED
#include <FastLED.h>

// Library to make buttons work
// See: https://github.com/mathertel/OneButton
#include <OneButton.h>

// --- CONFIGURATIONS --- //
// Pin configurations
#define SERVO_VERT_PIN 9
#define SERVO_HORIZ_PIN 10

#define AUX_PIN 4
#define PIXELS_PIN A0

#define BUTTON1_PIN 2

// Servo configurations
#define START_DEGREE_VALUE  90 // The degree value written to the servo at time of attach.
#define SERVO_SPEED 360
#define EASING_TYPE EASE_LINEAR // default

// Accelerometer configurations
// [TODO]

// Sound configurations
// [TODO]

// WS2812 Pixel configurations
#define PIXELS_NUM 7
#define PIXELS_TYPE WS2812

// Button configurations
// [TODO]

// --- CONFIGURATIONS END --- //

// --- GLOBAL VARIABLES --- //
// Instantiate the servo objects
ServoEasing ServoVert;
ServoEasing ServoHoriz;

// Instantiate the accelerometer/gyroscope object
MPU6050 mpu(Wire);
unsigned long mpuTimer = 0;

// Instantiate the array of pixels object
CRGB pixels[PIXELS_NUM];

// Instantiate the button object
OneButton button1(BUTTON1_PIN);

// --- GLOBAL VARIABLES END --- //

// --- INITIALIZATION --- //
void initServos(){
  Serial.println(F("Initializing servos..."));

  if (ServoVert.attach(SERVO_VERT_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo vertical"));
  }

  if (ServoHoriz.attach(SERVO_HORIZ_PIN, START_DEGREE_VALUE, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo vertical"));
  }

  ServoVert.setEasingType(EASING_TYPE);
  ServoHoriz.setEasingType(EASING_TYPE);

  setSpeedForAllServos(SERVO_SPEED);

  simDelayMillis(500);
}

void initGyro(){
  Serial.println(F("Initializing gyroscope"));
  
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  simDelayMillis(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void initAuxLed(){
  Serial.println(F("Initializing Aux LED..."));
  pinMode(AUX_PIN, OUTPUT);
}

void initPixels(){
  Serial.println(F("Initializing Pixels..."));
  FastLED.addLeds<WS2812, PIXELS_PIN, RGB>(pixels, PIXELS_NUM);
}

void initButtons(){
  Serial.println(F("Initializing Buttons..."));
  button1.attachClick(handle_Button1_Click);
  button1.attachDoubleClick(handle_Button1_DoubleClick);
  button1.attachLongPressStart(handle_Button1_LongPressStart);
  button1.attachLongPressStop(handle_Button1_LongPressStop);
  button1.attachDuringLongPress(handle_Button1_DuringLongPress);
}
// --- INITIALIZATION END --- //

// --- CORE FUNCTIONS --- //
void moveServos(int posVert, int posHoriz){
  ServoVert.setEaseTo(posVert, SERVO_SPEED);
  ServoHoriz.setEaseTo(posHoriz, SERVO_SPEED);
}

void getAngles(){
  mpu.update();
  
  if((millis() - mpuTimer) > 10){ // print data every 10ms
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
    mpuTimer = millis();  
  }
}

void auxLedOn(){
  digitalWrite(AUX_PIN, HIGH);
}

void auxLedOff(){
  digitalWrite(AUX_PIN, LOW);
}

void setPixels(int startPos, int endPos, CRGB color){
  for(int i = startPos; i < endPos; i++){
    pixels[i] = color;
  }
    
  FastLED.show();
}
// --- CORE FUNCTIONS END --- //

// --- SPECIAL EFFECTS --- //

// --- SPECIAL EFFECTS END --- //

// --- EVENT HANDLERS --- //
void handle_Button1_Click(){
  Serial.println(F("Button1 click..."));
  // TODO: Implement...
}

void handle_Button1_DoubleClick(){
  Serial.println(F("Button1 double click..."));
  // TODO: Implement...
}

void handle_Button1_LongPressStart(){
  Serial.println(F("Button1 long press start..."));
  // TODO: Implement...
}

void handle_Button1_LongPressStop(){
  Serial.println(F("Button1 long press stop..."));
  // TODO: Implement...
}

void handle_Button1_DuringLongPress(){
  Serial.println(F("Button1 during long press..."));
  // TODO: Implement...
}
// --- EVENT HANDLERS END --- //

// --- MONITORING FUNCTIONS --- //
void monitorButtons(){
  button1.tick();
}
// --- MONITORING FUNCTIONS --- //

/**
 * @brief Initialization method called by the Arduino library when the board boots up
 * 
 */
void setup(){
  Serial.begin(115200);

  initServos();

  initGyro();

  initAuxLed();

  initPixels();

  initButtons();

#ifdef RUN_UNIT_TESTS
  runTests();
#endif
}

/**
 * @brief Main program execution. This method will run perpetually on the board
 * 
 */
void loop(){
  monitorButtons();
}

// --- HELPER METHODS --- //

/**
 * Helper Method
 * Simulate a delay in processing without disabling the processor completely
 * 
 * @param[out] period - the amount of time in milliseconds to delay
 * @param[out] resolution -  MILLIS = milliseconds, MICROS = microseconds
 * 
 * See: https://randomnerdtutorials.com/why-you-shouldnt-always-use-the-arduino-delay-function/
*/
void simDelayMillis(unsigned long period){
  unsigned long delayMillis = millis() + period;
  while (millis() <= delayMillis)
  {
    byte x = 0; // dummy variable, does nothing
  }
}

/**
 * @brief Outputs the version of code to the serial monitor
 * 
 */
void printVersion(){
  Serial.print(F("Predator Servo Version: \t"));
  Serial.print(VERSION);
}

// --- HELPER METHODS END --- //

#ifdef RUN_UNIT_TESTS
// --- TEST METHODS --- //
void runTests(){
  test_moveServos();

  simDelayMillis(1000);

  test_AuxLedOnOff();

  simDelayMillis(1000);

  test_setPixels();

  simDelayMillis(1000);

  test_getAngles();
}

void test_moveServos(){
  Serial.println(F("Test moveServos()"));

  moveServos(0, 0);

  setEaseToForAllServosSynchronizeAndStartInterrupt();
  //while (!updateAllServos());

  simDelayMillis(2000);

  moveServos(180, 180);

  setEaseToForAllServosSynchronizeAndStartInterrupt();
  //while (!updateAllServos());

  simDelayMillis(2000);

  moveServos(90, 90);

  setEaseToForAllServosSynchronizeAndStartInterrupt();
  //while (!updateAllServos());

  Serial.println(F("Test moveServos() completed."));
}

void test_getAngles(){
  Serial.println(F("Test getAngles()"));

  for (int i = 0; i < 10000; i++){
    getAngles();
  }

  Serial.println(F("Test getAngles() completed."));
}

void test_AuxLedOnOff(){
  Serial.println(F("Test auxLedOn() | auxLedOff()"));

  for (int i = 0; i < 3; i++){
    auxLedOn();

    simDelayMillis(1000);

    auxLedOff();

    simDelayMillis(1000);
  }

  Serial.println(F("Test auxLedOn() | auxLedOff() completed."));
}

void test_setPixels(){
  Serial.println(F("Test setPixels()"));

  for (int i = 0; i < 3; i++){
    setPixels(0, PIXELS_NUM, CRGB::White);

    simDelayMillis(1000);

    setPixels(0, PIXELS_NUM, CRGB::Black);

    simDelayMillis(1000);
  }

  Serial.println(F("Test setPixels() completed."));
}
// --- TEST METHODS END --- //
#endif