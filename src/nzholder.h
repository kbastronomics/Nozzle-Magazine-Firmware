/**************************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 0.8.0B
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/Nozzle-Magazine-Firmware
 *
 * This Library is licensed under a GPLv3 License
 **************************************************************************************************/
#ifndef nzholder_h
#define nzholder_h

#include <Arduino.h>

#include <DRV8871.h> // https://github.com/dirkk1980/arduino_adafruit-drv8871
#include <gcode.h>   // https://github.com/tinkersprojects/G-Code-Arduino-Library
#include <Bounce2.h> // https://github.com/thomasfredericks/Bounce2

// To add current monitoring uncomment the #define __USE_INA219__ in the main file
#ifdef __ENABLE_INA219__
#include <Adafruit_INA219.h>

float total_mA;
unsigned long total_sec;
float shuntvoltage;
float busvoltage;
float current_mA;
float loadvoltage;
float power_mW;
float total_mAH;
float peekCurrent_mA;

Adafruit_INA219 ina219;

void read_ina219();
#endif

// TO add EEPROM support to store settings uncomment the #define __USE_EEPROM__ in the main file
#ifdef __ENABLE_EEPROM__

#endif

// FIRMWARE INFORMAITON
#define FIRMWARE_NAME       "Nozzle Magazine"
#define GITHUB_URL          "https://github.com/kbastronomics/Nozzle-Magazine-Firmware"
#define FIRMWARE_VERSION    "0.8.0B"
#define MACHINE_TYPE        "Deltaprintr Nozzle Magazine (large)"
#define NOZZLE_COUNT        20
#define MOTOR_DRIVER        "DRV8871"
#define CURRENT_MONITOR     "INA219"
#define CONTROL_BOARD       "ADAFRUIT_METRO_M0_EXPRESS"
#define UUID                "6fc71526-82e3-4c48-b30d-5c81313cd1fd"
#define ACTIVITYLED         LED_BUILTIN

//pin configuration for Adafruit Metro Express
#ifdef ADAFRUIT_METRO_M0_EXPRESS
#define MOTOR_IN1           9
#define MOTOR_IN2           10
#define LIMIT_CLOSE         11
#define LIMIT_OPEN          12
#ifdef __ENABLE_ESTOP_SWITCH__
    #define ESTOP_SWITCH        7
#endif
#define OPEN_BUTTON         2
#define CLOSE_BUTTON        3
#endif

//pin configuration for Adafruit Trinket M0
#ifdef ADAFRUIT_TRINKET_M0
#define MOTOR_IN1           4
#define MOTOR_IN2           5
#define LIMIT_CLOSE         2
#ifdef __ENABLE_ESTOP_SWITCH__
    #define ESTOP_SWITCH        0
#endif
#endif

//pin configuration for the KBAstronomic Nozzle Magazine Control Board
#ifdef KBASTRO_NOZZLE_MAGAZINE
#define MOTOR_IN1           4
#define MOTOR_IN2           5
#define LIMIT_CLOSE         2
#define LIMIT_OPEN          3
#define CURRENT_SENSE       6
#ifdef __ENABLE_ESTOP_SWITCH__
    #define ESTOP_SWITCH        0
#endif
#define OPEN_BUTTON         7
#define CLOSE_BUTTON        8
#endif

// declarations
int checkParms();                   // SUPPORT FUNCTION TO READ COMMON PARMS               
void G4_pause();                    // G4 S<VALUE>
void M111_debug();                  // M111 P<MODULE> L<LEVEL>
void M112_estop();                  // M112
void M114_reportPostion();          // M114 Report Postion
void M115_reportFirmware();         // M115 Report Firmware Information
void M119_endStopState();           // M119 Report Endstop State
void M220_setFeedrate();            // M220 S<VALUE> A<VALUE> D<VALUE>
void M303_autotune();               // ATTEMPT TO CREATE A AUTOTUNING FUNCTION
void M804_openNozzleMagazine();     // M804 S<VALUE> A<VALUE> D<VALUE>
void M805_closeNozzleMagazine();    // M805 S<VALUE> A<VALUE> D<VALUE>
void T1_test(); // Dummy Code to do some timing tests
void openNozzleMagazine(); // Manual override to open nozzle
void closeNozzleMagazine(); // Manual override to close nozzle

DRV8871 deltaprintr_motor(
  MOTOR_IN1,
  MOTOR_IN2
);

int _speed = 220; // Max value to not stall @ 12V VMotor,  254 @ 9V VMotor 
int _acceleration = 0; // No Acceleration Delay
int _brake = 0; // STOP IMMEDIATE
int _debug_module = 0; // P<MODEULE> where P=G/M Code
int _debug_level = 0; // S<LEVEL> where 0=OFF, 1=ON
int _limitClosed = HIGH; // Using ISR to set Limit instead of a digitalread
int _limitOpen = HIGH; // Using ISR to set Limit instead of a digitalread
int _estop = HIGH; // E-STOP PIN
unsigned long _timeout = 2000; // sdefault timeout value of 2 seconds

#define NumberOfCommands 11

commandscallback commands[NumberOfCommands] = {
  {
    "G4",
    G4_pause
  }, // G4 P<VALUE>  IGNORED
  {
    "T1",
    T1_test
  }, // T1 
  {
    "M111",
    M111_debug
  }, // M111 
  {
    "M112",
    M112_estop
  }, // M112
  {
    "M114",
    M114_reportPostion
  }, // M114
  {
    "M115",
    M115_reportFirmware
  }, // M115
  {
    "M119",
    M119_endStopState
  }, // M119
  {
    "M220",
    M220_setFeedrate
  }, // M220 S<PERCENT> A<MILLISECONDS> B<VALUE>
  {
    "M303",
    M303_autotune
  }, // M303 C<COUNT> DEFAULTS IS 5
  {
    "M804",
    M804_openNozzleMagazine
  }, // M804 S<PERCENT> AMILLISECONDS> B<VALUE>
  {
    "M805",
    M805_closeNozzleMagazine
  } // M805 S<PERCENT> A<MILLISECONDS> B<VALUE>
};

gcode GCode(NumberOfCommands, commands);

Bounce2::Button open_button = Bounce2::Button();
Bounce2::Button close_button = Bounce2::Button();

// TO add EEPROM support to store settings uncomment the #define __USE_EEPROM__ in the main file
#ifdef __ENABLE_ESTOP_SWITCH__
Bounce2::Button estop_button = Bounce2::Button();
#endif

#endif

// SAMPLE INIT STRING
// GET FIRNWARE INFO, SET SPEED/ACCEL/BREAK, Check Position and Make sure closed
// M115 M220 S220 A0 B0 M114 M805