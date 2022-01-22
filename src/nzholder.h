/**************************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 1.0.0
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/Nozzle-Magazine-Firmware
 *
 * This Library is licensed under a GPLv3 License
 **************************************************************************************************/
#ifndef nzholder_h
#define nzholder_h

#include <Arduino.h>

#include <DRV8871.h> // https://github.com/dirkk1980/arduino_adafruit-drv8871
#include <gcode.h>   // https://github.com/tinkersprojects/G-Code-Arduino-Library


// FIRMWARE/CONTROLER  INFORMAITON
#define FIRMWARE_NAME       "Nozzle Magazine"
#define GITHUB_URL          "https://github.com/kbastronomics/Nozzle-Magazine-Firmware"
#define FIRMWARE_VERSION    "1.0.0"
#ifdef __LARGE_NOZZLE__
    #define MACHINE_TYPE        "Deltaprintr Nozzle Magazine (Large)"
    #define NOZZLE_COUNT        20
#endif
#ifdef __SMALL_NOZZLE__
    #define MACHINE_TYPE        "Deltaprintr Nozzle Magazine (Small)"
    #define NOZZLE_COUNT        10
#endif
#define CONTROL_BOARD       "ADAFRUIT_METRO_M0_EXPRESS"
#define MOTOR_DRIVER        "DRV8871"
#define CURRENT_MONITOR     "INA219"
#define UUID                "6fc71526-82e3-4c48-b30d-5c81313cd1fd"
#define NUMBER_MAGAZINES    1

//pin configuration for Adafruit Metro Express
#ifdef ADAFRUIT_METRO_M0_EXPRESS
    #define MOTOR_IN1               9
    #define MOTOR_IN2               10
    #define LIMIT_CLOSE             11
    #define LIMIT_OPEN              12
    #ifdef __ENABLE_ESTOP_SWITCH__
      #define ESTOP_SWITCH        7
    #endif
    #ifdef __ENABLE_OC_SWITCH__
      #define OPEN_BUTTON             2
      #define CLOSE_BUTTON            3
    #endif
    #ifdef __ENABLE_NEOPIXEL__
      #define NEOPIXEL_PIN        40
    #endif 
    #define ACTIVITYLED             LED_BUILTIN   // LED_BUILTIN PIN_LED_13  (13u)
#endif

//pin configuration for Adafruit Trinket M0 
//fewer IO pins so can't do everything at same time
#ifdef ADAFRUIT_TRINKET_M0
    #define MOTOR_IN1               4
    #define MOTOR_IN2               5
    #define LIMIT_CLOSE             2
    #define LIMIT_OPEN              3
    #ifdef __ENABLE_ESTOP_SWITCH__
        #define ESTOP_SWITCH        0
    #endif
    #define ACTIVITYLED             LED_BUILTIN
    #ifdef __ENABLE_NEOPIXEL__
        #define NEOPIXEL_PIN        NEOPIXEL_BUILTIN
    #endif 
#endif

// Just to make code more readable
#define CLOSED  false
#define OPEN    true

// declarations
int checkParms();                   // SUPPORT FUNCTION TO READ COMMON PARMS               
void G4_pause();                    // G4 S<VALUE>
void M42_pinstate();                // M42 - Set Pin State / M42 [M<0|1|2|3>] [P<pin>] S<state>
void T100_test();                   // M100 - Print Tests
void M111_debug();                  // M111 - Debug Level / M111 [S<flags>]
void M112_estop();                  // M112 - Emergency Stop / M112 [P<level>]
void M114_reportPostion();          // M114 - Get Current Position / M114 [D] [E] [R]
void M115_reportFirmware();         // M115 - Firmware Info / M115
void M119_endStopState();           // M119 - Endstop States / M119
void M220_setFeedrate();            // M220 - Set Feedrate Percentage / M220 S<VALUE> A<VALUE> D<VALUE>
void M303_autotune();               // M303 - PID autotune / M303 C<count> D<action> D<flag> [E<index>] S<temp> U<flag> (TBD)
void M804_openNozzleMagazine();     // M804 - Open Nozzle Magazine / M804 S<VALUE> A<VALUE> D<VALUE>
void M805_closeNozzleMagazine();    // M805 - Close Nozzle Magazine / M805 S<VALUE> A<VALUE> D<VALUE>

// GLOBAL VARIABLES

int iSpeed = 220; // Max value to not stall @ 12V VMotor,  254 @ 9V VMotor 
int iAcceleration = 0; // No Acceleration Delay
int iBrake = 0; // STOP IMMEDIATE
int iDebugmodule = 0; // P<MODEULE> where P=G/M Code
int iDebuglevel = 0; // S<LEVEL> where 0=OFF, 1=ON
unsigned long ulTimeout = 2000; // sdefault timeout value of 2 seconds
bool bEstop = false;
bool bError = false; 
bool bWaiting = false;
bool bOCstate = CLOSED;

DRV8871 deltaprintr_motor(
  MOTOR_IN1,
  MOTOR_IN2
);


#define NumberOfCommands 12


commandscallback commands[NumberOfCommands] = {
  {
    "T100",
    T100_test
  }, // 
  {
    "G4",
    G4_pause
  }, // G4 P<VALUE>  
  {
    "M42",
    M42_pinstate
  }, // M43
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

// To add current monitoring uncomment the #define __USE_INA219__ in the main file
#ifdef __ENABLE_INA219__
    #include <Wire.h>
    #include <INA219_WE.h>  // https://github.com/wollewald/INA219_WE
                            // https://learn.adafruit.com/adafruit-metro-m0-express/pinouts
    #define I2C_ADDRESS 0x40

    void read_ina219(); // declare the function since its now being used

    INA219_WE ina219 = INA219_WE(I2C_ADDRESS);

    // define some global variables.   Maybe move into a structure instead???
    float fTotalmA;
    unsigned long ulTotalsec;
    float fShuntvoltage;
    float fBusvoltage;
    float fCurrentmA;
    float fLoadvoltage;
    float fPowermW;
    float fTotalmAH;
    float fPeekCurrentmA;
    bool bIna219overflow = false;
#endif

// Add support files for the NEOPIXEL Status LED
#ifdef __ENABLE_NEOPIXEL__
    #include <Adafruit_NeoPixel.h>

    void neopixel_led(uint8_t color);

    int iBrightness = 200;  
    int iPulsespeed = 2000;  // default 2000 slow pulse for blue waiting,  1000 for red estop

    #define NUMPIXELS       1
    #define NEOPIXEL_RED    1
    #define NEOPIXEL_GREEN  2
    #define NEOPIXEL_BLUE   3
    #define NEOPIXEL_WHITE  4
    #define NEOPIXEL_AMBER  5
    #define NEOPIXEL_YELLOW 6
    #define NEOPIXEL_ON     iBrightness
    #define NEOPIXEL_OFF    0

    Adafruit_NeoPixel strip(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

    // CODE TO SUPPORT PULSING NEOPIXEL
    uint8_t strip0_loop0_eff0();
    uint8_t strip0_loop0_eff1();
    uint8_t strip0_loop0();
    void strips_loop();
    void neopixel_led(uint8_t color, uint8_t brightness);

    class Strip
    {
    public:
      uint8_t   effect;
      uint8_t   effects;
      uint16_t  effStep;
      unsigned long effStart;
      Adafruit_NeoPixel strip;
      Strip(uint16_t leds, uint8_t pin, uint8_t toteffects, uint16_t striptype) : strip(leds, pin, striptype) {
        effect = -1;
        effects = toteffects;
        Reset();
      }
      void Reset(){
        effStep = 0;
        effect = (effect + 1) % effects;
        effStart = millis();
      }
    };

    struct Loop
    {
      uint8_t currentChild;
      uint8_t childs;
      bool timeBased;
      uint16_t cycles;
      uint16_t currentTime;
      Loop(uint8_t totchilds, bool timebased, uint16_t tottime) {currentTime=0;currentChild=0;childs=totchilds;timeBased=timebased;cycles=tottime;}
    };

    Strip strip_0(1, NEOPIXEL_PIN, 1, NEO_GRB + NEO_KHZ800);
    struct Loop strip0loop0(2, false, 1);

#endif

// TO add EEPROM support to store settings uncomment the #define __USE_EEPROM__ in the main file
#ifdef __ENABLE_EEPROM__

#endif

#ifdef __ENABLE_OC_SWITCH__
    #include <Bounce2.h> // https://github.com/thomasfredericks/Bounce2
    //static bool oc_state = 0;    
    #define __NUM_OC_BUTTONS__ 2 

    void openNozzleMagazine(); // Manual override to open nozzle
    void closeNozzleMagazine(); // Manual override to close nozzle

    // Create open/close buttons
    Bounce2::Button Openswitch = Bounce2::Button();
    Bounce2::Button Closeswitch = Bounce2::Button();
#endif 

// TO add EEPROM support to store settings uncomment the #define __USE_EEPROM__ in the main file
#ifdef __ENABLE_ESTOP_SWITCH__
    Bounce2::Button Estopswitch = Bounce2::Button();
#endif

#endif
// SAMPLE INIT STRING
// GET FIRNWARE INFO, SET SPEED/ACCEL/BREAK, Check Position and Make sure closed
// M115 M220 S220 A0 B0 M114 M805