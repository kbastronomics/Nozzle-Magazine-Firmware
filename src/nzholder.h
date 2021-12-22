/**********************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 1.0
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/smgvbest/nzmag
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <Arduino.h>
#include <DRV8871.h> // https://github.com/dirkk1980/arduino_adafruit-drv8871
#include <gcode.h>   // https://github.com/tinkersprojects/G-Code-Arduino-Library

// FIRMWARE INFORMAITON
#define FIRMWARE_NAME     "Nozzle Magazine"
#define FIRMWARE_VERSION   1.0.0
#define NOZZLE_COUNT      20
//#define NOZZLE_COUNT 10
#define PROTOCOL_VERSION 1.0
#define MACHINE_TYPE "Deltaprintr nozzle magazinw"

#define LEDpin LED_BUILTIN


//pin configuration for Adafruit Metro Express
#ifdef ADAFRUIT_METRO_M0_EXPRESS
#define MOTOR_IN1   9
#define MOTOR_IN2   10
#define LIMIT_CLOSE 11
#define LIMIT_OPEN  12
#define ESTOP       13
#endif

//pin configuration for Adafruit Trinket M0
#ifdef ADAFRUIT_TRINKET_M0
#define MOTOR_IN1   0
#define MOTOR_IN2   1
#define LIMIT_CLOSE 2
#define LIMIT_OPEN  3
#define ESTOP       4
#endif

// declarations
int checkAS();              // SUPPORT FUNCTION
void estop();               // M112
void reportPostion();       // M114
void reportFirmware();      // M115
void endStopState();        // M119
void setFeedrate();         // M220
void openNozzleholdder();   // M804
void closeNozzleholdder();  // M805

DRV8871 deltaprintr_motor(
    MOTOR_IN1,
    MOTOR_IN2);

int nSpeed = 75;          // NOT FASTEST
int nAcceleration = 10;   // QUICK ACCEL
int nBreak=0;             // STOP IMMEDIATE

#define NumberOfCommands 7

commandscallback commands[NumberOfCommands] = {
    {"M112", estop},             // M112
    {"M114", reportPostion},     // M114
    {"M115", reportFirmware},    // M115
    {"M119", endStopState},      // M119
    {"M220", setFeedrate},       // M220 S<PERCENT> A<PERCENT> B<VALUE>
    {"M804", openNozzleholdder}, // M804 S<PERCENT> A<PERCENT> B<VALUE>
    {"M805", closeNozzleholdder} // M805 S<PERCENT> A<PERCENT> B<VALUE>
};

// SAMPLE INIT STRING
// M115 M220 S30 A10 B0 M114 
gcode GCode(NumberOfCommands, commands);