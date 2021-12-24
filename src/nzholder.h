/**********************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 0.2.0b
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/nzmag
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <Arduino.h>
#include <DRV8871.h> // https://github.com/dirkk1980/arduino_adafruit-drv8871
#include <gcode.h>   // https://github.com/tinkersprojects/G-Code-Arduino-Library

// SET DEBUG TO 1 FOR ADDITIONAL MESSAGES
#define DEBUG 0


// FIRMWARE INFORMAITON
#define FIRMWARE_NAME       "Nozzle Magazine"
#define FIRMWARE_VERSION    "0.2.0b"
#define MACHINE_TYPE        "Deltaprintr nozzle magazine (large)"
#define NOZZLE_COUNT        20
#define PROTOCOL_VERSION    "1.0"
#define MOTOR_DRIVE         "DRV8871"
#define CONTROL_BOARD       "ADAFRUIT_METRO_M0_EXPRESS"
#define UUID                "6fc71526-82e3-4c48-b30d-5c81313cd1fd"

#define TIMEOUT             2000
#define LEDpin              LED_BUILTIN


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
#define MOTOR_IN1   4
#define MOTOR_IN2   5
#define LIMIT_CLOSE 2
#define LIMIT_OPEN  3
#define E-STOP      0
#endif

// declarations
int checkASB();                   // SUPPORT FUNCTION
void G4_pause();                  // G4 S<VALUE>
void M111_debug();                // M111 P<MODULE> S<LEVEL>
void M112_estop();                // M112
void M114_reportPostion();        // M114
void M115_reportFirmware();       // M115
void M119_endStopState();         // M119
void M220_setFeedrate();          // M220 S<VALUE> A<VALUE> D<VALUE>
void M303_autotune();             // ATTEMPT TO CREATE A AUTOTUNING FUNCTION
void M804_openNozzlemagazine();   // M804 S<VALUE> A<VALUE> D<VALUE>
void M805_closeNozzlemagazine();  // M805 S<VALUE> A<VALUE> D<VALUE>

DRV8871 deltaprintr_motor(
    MOTOR_IN1,
    MOTOR_IN2);

int nSpeed = 1000;          // Gets most torque
int nAcceleration = 0;      // No Acceleration Delay
int nBreak=0;               // STOP IMMEDIATE
int debug_module=0;         // P<MODEULE> where P=G/M Code
int debug_level=0;          // S<LEVEL> where 0=OFF, 1=ON

#define NumberOfCommands 10

commandscallback commands[NumberOfCommands] = {
    {"G4", G4_pause},                  // G4 P<VALUE>  IGNORED
    {"M111", M111_debug},              // M111 
    {"M112", M112_estop},              // M112
    {"M114", M114_reportPostion},      // M114
    {"M115", M115_reportFirmware},     // M115
    {"M119", M119_endStopState},       // M119
    {"M220", M220_setFeedrate},        // M220 S<PERCENT> A<MILLISECONDS> B<VALUE>
    {"M303", M303_autotune},           // M303 C<COUNT> DEFAULTS IS 5
    {"M804", M804_openNozzlemagazine}, // M804 S<PERCENT> AMILLISECONDS> B<VALUE>
    {"M805", M805_closeNozzlemagazine}// M805 S<PERCENT> A<MILLISECONDS> B<VALUE>
};

// SAMPLE INIT STRING
// GET FIRNWARE INFO, SET SPEED/ACCEL/BREAK, Check Position and Make sure closed
// M115 M220 S1000 A0 B0 M114 M805

gcode GCode(NumberOfCommands, commands);