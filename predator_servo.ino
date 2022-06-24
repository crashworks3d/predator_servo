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

// Uncomment this line to enable sound
#define SOUND

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
#ifdef SOUND
// See: https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299#target_6
// Important!!! On the SD card copy the mp3 files into an mp3 directory
// Download and install the DFRobotDFPlayerMini library

#include <DFPlayer.h>
#include <SoftwareSerial.h>
#endif

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

#define EYE_LEFT_PIN 6
#define EYE_RIGHT_PIN 3

#define AUX_PIN 4
#define PIXELS_PIN A0

#define BUTTON1_PIN 2

// Servo configurations
#define START_DEGREE_VALUE  90 // The degree value written to the servo at time of attach.
#define REST_DEGREE_VALUE_VERT 0 // The degree value written to the vertical servo at rest.
#define REST_DEGREE_VALUE_HORIZ 90 // The degree value written to the horizontal servo at rest.
#define SERVO_SPEED 360 // The speed of the servo in degrees/ms.
#define EASING_TYPE EASE_LINEAR // The easing type servo motion, default is EASE_LINEAR.

// Accelerometer configurations
#define MPU_SAMPLE_RATE 10 // Frequency in milliseconds on how much time passes between getting values

// Sound configurations
#ifdef SOUND
// sound board pins
#define RX_PIN 7 // set pin for receive (RX) communications
#define TX_PIN 8 // set pin for transmit (TX) communications
#define VOLUME 29 // sound board volume level (30 is max)
#define MP3_TYPE DFPLAYER_MINI // Chip type of DFPlayerMini (see documentation)
#define MP3_SERIAL_TIMEOUT 100 //average DFPlayer response timeout 100msec..200msec

// TODO: Sound effects for predator
#define SND_CANNON 1 // sound track for helmet closing sound
#define SND_GROWL 2 // sound track for JARVIS sound
#define SND_ROAR 3 // sound track for helmet opening sound
#endif

// WS2812 Pixel configurations
#define PIXELS_NUM 7
#define PIXELS_COLOR CRGB(253,255,249)
#define PIXELS_TYPE WS2812

// LED eye configurations
#define EYE_LED_MAX_BRIGHTNESS 200 // Range 0-255 (PWM)

// Button configurations
// [TODO]

// --- CONFIGURATIONS END --- //

// --- GLOBAL VARIABLES --- //
// Instantiate the servo objects
ServoEasing ServoVert;
ServoEasing ServoHoriz;

// Map degrees to float
using degree = float;
// Create a struct to hold gyro values
struct Angles {
  degree pitch;
  degree roll;
  degree yaw;
};

// Object to hold current angles values
Angles curAngles = { 0, 0, 0 };
Angles prevAngles = { 0, 0, 0 };

// Create a struct to hold x, y servo degree values
struct ServoPos {
  int vertical;
  int horizontal;
};

// Object to hold current servo position values
ServoPos curServoPos = { 0, 0 };
ServoPos prevServoPos = { 0, 0 };

// Instantiate the accelerometer/gyroscope object
MPU6050 gyroObj(Wire);
unsigned long mpuTimer = 0;

boolean isGyroActive = false;

// Instantiate the array of pixels object
CRGB pixels[PIXELS_NUM];

// Instantiate the button object
OneButton button1(BUTTON1_PIN);

#ifdef SOUND
// Declare variables for sound control
SoftwareSerial serialObj(RX_PIN, TX_PIN); // Create object for serial communications
DFPlayer mp3Obj; // Create object for DFPlayer Mini
#endif

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
  Serial.println(F("Initializing gyroscope..."));
  
  Wire.begin();

  byte status = 0;
  gyroObj.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  simDelayMillis(1000);
  // gyroObj.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  
  gyroObj.calcOffsets(); // gyro and accelerometer

  prevAngles = getAngles();

  Serial.println("Done!\n");

  printGyroOffsets();
}

#ifdef SOUND
/**
 * Initialization method for DFPlayer Mini board
 */
 void initPlayer(){
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  serialObj.begin(9600);

  mp3Obj.begin(serialObj, MP3_SERIAL_TIMEOUT, MP3_TYPE, false);

  mp3Obj.stop();        //if player was runing during ESP8266 reboot
  mp3Obj.reset();       //reset all setting to default
  
  mp3Obj.setSource(2);  //1=USB-Disk, 2=TF-Card, 3=Aux, 4=Sleep, 5=NOR Flash
  
  mp3Obj.setEQ(0);      //0=Off, 1=Pop, 2=Rock, 3=Jazz, 4=Classic, 5=Bass
  mp3Obj.setVolume(VOLUME); //0..30, module persists volume on power failure

  mp3Obj.sleep();       //inter sleep mode, 24mA
 }
 #endif

void initEyeLeds(){
  Serial.println(F("Initializing LED eyes..."));
  pinMode(EYE_LEFT_PIN, OUTPUT);
  pinMode(EYE_RIGHT_PIN, OUTPUT);
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
  Serial.println(F("Moving servos..."));

  ServoVert.setEaseTo(posVert, SERVO_SPEED);
  ServoHoriz.setEaseTo(posHoriz, SERVO_SPEED);
}

void moveServos(ServoPos postion){
  moveServos(postion.vertical, postion.horizontal);
}

void detachServos(){
  ServoVert.detach();
  ServoHoriz.detach();
}

Angles getAngles(){
  gyroObj.update();

  Angles angles;
  angles.pitch = gyroObj.getAngleX();
  angles.roll = gyroObj.getAngleY();
  angles.yaw = gyroObj.getAngleZ();

  return angles;
}

ServoPos getServoPositons(){
  ServoPos servoPos;

  // Compensate for gyro drift over usage
  // NOTE: rapid movements (i.e. big accelerometer actions) tend to increase drift...
  curAngles.pitch = prevAngles.pitch >= 0 ? curAngles.pitch - prevAngles.pitch : curAngles.pitch + prevAngles.pitch;
  curAngles.yaw = prevAngles.yaw >= 0 ? curAngles.yaw - prevAngles.yaw : curAngles.yaw + prevAngles.yaw;

  // Map the gyro readings to servo positions in degrees 0-180
  servoPos.vertical = map(curAngles.pitch, -90, 90, 180, 0);
  servoPos.horizontal = map(curAngles.yaw, -90, 90, 0, 180);

  // Stay within the boundaries of 0 to 180 degrees
  servoPos.vertical = servoPos.vertical <= 180 ? servoPos.vertical : 180;
  servoPos.vertical = servoPos.vertical >= 0 ? servoPos.vertical : 0;
  servoPos.horizontal = servoPos.horizontal <= 180 ? servoPos.horizontal : 180;
  servoPos.horizontal = servoPos.horizontal >= 0 ? servoPos.horizontal : 0;

  return servoPos;
}

#ifdef SOUND
/**
 * Method to play the sound effect for a specified feature
 */
void playSoundEffect(int soundEffect){
  Serial.print(F("Playing sound effect: "));
  Serial.println(soundEffect);

  mp3Obj.wakeup(2);

  mp3Obj.playTrack(soundEffect);
}
#endif

void setLedEyes(int pwmValue){
  analogWrite(EYE_LEFT_PIN, pwmValue);
  analogWrite(EYE_RIGHT_PIN, pwmValue);
}

void eyeLedsOn(){
  setLedEyes(EYE_LED_MAX_BRIGHTNESS);
}

void eyeLedsOff(){
  setLedEyes(0);
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
void moveServosToRestPosition(){
  int restVert = 0;
  int restHoriz = 90;

  moveServos(restVert, restHoriz);

  setEaseToForAllServosSynchronizeAndStartInterrupt();
}

void trackTarget(){
  curAngles = getAngles();
  curServoPos = getServoPositons();

  if (prevServoPos.vertical != curServoPos.vertical){
    moveServos(curServoPos);
    prevServoPos.vertical = curServoPos.vertical;
  }

  if (prevServoPos.horizontal != curServoPos.horizontal){
    moveServos(curServoPos);
    prevServoPos.horizontal = curServoPos.horizontal;
  }

  setEaseToForAllServosSynchronizeAndStartInterrupt();

  while (!updateAllServos()){
    //
  }
}

// --- SPECIAL EFFECTS END --- //

// --- EVENT HANDLERS --- //
void handle_Button1_Click(){
  Serial.println(F("Button1 click..."));
  // TODO: Implement...

  isGyroActive = !isGyroActive; // Toggle activating gyroscope on/off

  if (isGyroActive){
    Serial.println(F("Target tracking active..."));

    // Activate servos and put them in initial position
    initServos();

    // Initialize the gyroscope and calibrate
    initGyro();
  } else {
    moveServosToRestPosition();

    detachServos();
  }
}

void handle_Button1_DoubleClick(){
  Serial.println(F("Button1 double click..."));
  // TODO: Implement...
  initGyro();
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

void monitorGyro(){
  if(isGyroActive){
    if((millis() - mpuTimer) > MPU_SAMPLE_RATE){
      curAngles = getAngles();

      printAngles();

      curServoPos = getServoPositons();

      // printServoPos();

      trackTarget();

      mpuTimer = millis();
    }
  }
}
// --- MONITORING FUNCTIONS --- //

/**
 * @brief Initialization method called by the Arduino library when the board boots up
 * 
 */
void setup(){
  Serial.begin(115200);

  initPlayer();

  initServos();

  initGyro();

  initEyeLeds();

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

  monitorGyro();
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

/**
 * @brief Outputs the values of the current angles measured on the gyro
 * 
 */
void printAngles(){
  Serial.print(F("{ pitch: "));
  Serial.print(curAngles.pitch);
  Serial.print(F(", roll: "));
  Serial.print(curAngles.roll);
  Serial.print(F(", yaw: "));
  Serial.print(curAngles.yaw);
  Serial.println(" }");
}

void printServoPos(){
  Serial.print(F("vertical: "));
  Serial.print(curServoPos.vertical);
  Serial.print(F("\thorizontal: "));
  Serial.println(curServoPos.horizontal);
}

void printGyroOffsets(){
  Serial.print(F("AccXoffset: "));
  Serial.print(gyroObj.getAccXoffset());
  Serial.print(F("\tAccYoffset: "));
  Serial.print(gyroObj.getAccYoffset());
  Serial.print(F("\tAccZoffset: "));
  Serial.print(gyroObj.getAccZoffset());

  Serial.print(F("\tGyroXoffset: "));
  Serial.print(gyroObj.getGyroXoffset());
  Serial.print(F("\tGyroYoffset: "));
  Serial.print(gyroObj.getGyroYoffset());
  Serial.print(F("\tGyroZoffset: "));
  Serial.println(gyroObj.getGyroZoffset());
}

// --- HELPER METHODS END --- //

#ifdef RUN_UNIT_TESTS
// --- TEST METHODS --- //
void runTests(){
  test_moveServos();

  simDelayMillis(1000);

  test_moveServosToRestPosition();

  simDelayMillis(1000);

  test_playSoundEffect();

  simDelayMillis(1000);

  test_ledEyesOnOff();

  simDelayMillis(1000);

  test_AuxLedOnOff();

  simDelayMillis(1000);

  test_setPixels();

  simDelayMillis(1000);

  test_getAngles();

  simDelayMillis(1000);

  test_getServoPositons();
}

void test_moveServos(){
  Serial.println(F("Test moveServos()"));

  moveServos(0, 0);

  setEaseToForAllServosSynchronizeAndStartInterrupt();

  simDelayMillis(2000);

  moveServos(180, 180);

  setEaseToForAllServosSynchronizeAndStartInterrupt();

  simDelayMillis(2000);

  moveServos(90, 90);

  setEaseToForAllServosSynchronizeAndStartInterrupt();

  simDelayMillis(500);

  detachServos();

  Serial.println(F("Test moveServos() completed."));
}

void test_moveServosToRestPosition(){
  Serial.println(F("Test moveServosToRestPosition()"));

  initServos();

  moveServosToRestPosition();

  simDelayMillis(2000);

  moveServos(90, 90);

  setEaseToForAllServosSynchronizeAndStartInterrupt();

  simDelayMillis(500);

  detachServos();

  Serial.println(F("Test moveServosToRestPosition() completed."));
}

void test_getAngles(){
  Serial.println(F("Test getAngles()"));

  for (int i = 0; i < 100; i++){
    curAngles = getAngles();

    printAngles();

    simDelayMillis(10);
  }

  simDelayMillis(200);

  Serial.println(F("Test getAngles() completed."));
}

void test_getServoPositons(){
  Serial.println(F("Test getServoPositons()"));

  for (int i = 0; i < 100; i++){
    curAngles = getAngles();

    printAngles();

    curServoPos = getServoPositons();

    printServoPos();

    simDelayMillis(10);
  }

  simDelayMillis(200);

  Serial.println(F("Test getServoPositons() completed."));
}

void test_playSoundEffect(){
  Serial.println(F("Test playSoundEffect()"));

  // Make sure there is at least 1 mp3 file on SD card
  playSoundEffect(1);

  simDelayMillis(2000);

  Serial.println(F("Test playSoundEffect() completed."));
}

void test_ledEyesOnOff(){
  Serial.println(F("Test ledEyesOn() | ledEyesOff()"));

  for (int i = 0; i < 3; i++){
    eyeLedsOn();

    simDelayMillis(1000);

    eyeLedsOff();

    simDelayMillis(1000);
  }

  Serial.println(F("Test ledEyesOn() | ledEyesOff() completed."));
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
    setPixels(0, PIXELS_NUM, PIXELS_COLOR);

    simDelayMillis(1000);

    setPixels(0, PIXELS_NUM, CRGB::Black);

    simDelayMillis(1000);
  }

  Serial.println(F("Test setPixels() completed."));
}
// --- TEST METHODS END --- //
#endif