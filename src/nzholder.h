/**********************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 1.0
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/nzmag
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
int checkASB();              // SUPPORT FUNCTION
void M112_estop();               // M112
void M114_reportPostion();       // M114
void M115_reportFirmware();      // M115
void M119_endStopState();        // M119
void M220_setFeedrate();         // M220
void M804_openNozzleholdder();   // M804
void M805_closeNozzleholdder();  // M805

DRV8871 deltaprintr_motor(
    MOTOR_IN1,
    MOTOR_IN2);

int nSpeed = 75;          // NOT FASTEST
int nAcceleration = 10;   // QUICK ACCEL
int nBreak=0;             // STOP IMMEDIATE

#define NumberOfCommands 7

commandscallback commands[NumberOfCommands] = {
    {"M112", M112_estop},             // M112
    {"M114", M114_reportPostion},     // M114
    {"M115", M115_reportFirmware},    // M115
    {"M119", M119_endStopState},      // M119
    {"M220", M220_setFeedrate},       // M220 S<PERCENT> A<PERCENT> B<VALUE>
    {"M804", M804_openNozzleholdder}, // M804 S<PERCENT> A<PERCENT> B<VALUE>
    {"M805", M805_closeNozzleholdder} // M805 S<PERCENT> A<PERCENT> B<VALUE>
};

// SAMPLE INIT STRING
// M115 M220 S30 A10 B0 M114 
gcode GCode(NumberOfCommands, commands);