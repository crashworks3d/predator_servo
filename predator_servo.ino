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

// Referenced libraries
// [TODO: Servo]

// [TODO: Accelerometer]

// Library to make sound work
// See: https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299#target_6
// Important!!! The SD card needs to be formatted correctly as FAT32 or FAT16, and all MP3 files need to be named numerically 001 ... 002 ... 003
// Download and install the DFRobotDFPlayerMini library
#include <DFRobotDFPlayerMini.h>

// [TODO: WS2812]

// [TODO: Button]

// --- CONFIGURATIONS --- //
// Pin configurations
// [TODO]

// Servo configurations
// [TODO]

// Accelerometer configurations
// [TODO]

// Sound configurations
// [TODO]

// WS2812 Pixel configurations
// [TODO]

// Button configurations
// [TODO]

// --- CONFIGURATIONS END --- //

// --- GLOBAL VARIABLES --- //

// --- GLOBAL VARIABLES END --- //

// --- INITIALIZATION --- //

// --- INITIALIZATION END --- //

// --- CORE FUNCTIONS --- //

// --- CORE FUNCTIONS END --- //

// --- SPECIAL EFFECTS --- //

// --- SPECIAL EFFECTS END --- //

// --- EVENT HANDLERS --- //

// --- EVENT HANDLERS END --- //

/**
 * @brief Initialization method called by the Arduino library when the board boots up
 * 
 */
void setup(){

}

/**
 * @brief Main program execution. This method will run perpetually on the board
 * 
 */
void loop(){

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
 * @brief Method to output any issues with the DFPlayer
 * 
 * @param type 
 * @param value 
 */
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
// --- HELPER METHODS END --- //