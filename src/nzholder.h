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
void M111_debug();                  // M111 P<MODULE> L<LEVEL>
void M112_estop();                  // M112
void M114_reportPostion();          // M114 Report Postion
void M115_reportFirmware();         // M115 Report Firmware Information
void M119_endStopState();           // M119 Report Endstop State
void M220_setFeedrate();            // M220 S<VALUE> A<VALUE> D<VALUE>
void M303_autotune();               // ATTEMPT TO CREATE A AUTOTUNING FUNCTION
void M804_openNozzleMagazine();     // M804 S<VALUE> A<VALUE> D<VALUE>
void M805_closeNozzleMagazine();    // M805 S<VALUE> A<VALUE> D<VALUE>

// GLOBAL VARIABLES

int _speed = 220; // Max value to not stall @ 12V VMotor,  254 @ 9V VMotor 
int _acceleration = 0; // No Acceleration Delay
int _brake = 0; // STOP IMMEDIATE
int _debug_module = 0; // P<MODEULE> where P=G/M Code
int _debug_level = 0; // S<LEVEL> where 0=OFF, 1=ON
unsigned long _timeout = 2000; // sdefault timeout value of 2 seconds
bool __estop__ = false;
bool __error__ = false; 
bool __sleeping__ = false;
bool __oc_state__ = CLOSED;

DRV8871 deltaprintr_motor(
  MOTOR_IN1,
  MOTOR_IN2
);


#define NumberOfCommands 10


commandscallback commands[NumberOfCommands] = {
  {
    "G4",
    G4_pause
  }, // G4 P<VALUE>  IGNORED
#ifdef __ENABLE_TEST_CODE__
  {
    "T1",
    T1_test
  }, // T1 
#endif 
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
    float total_mA;
    unsigned long total_sec;
    float shuntvoltage;
    float busvoltage;
    float current_mA;
    float loadvoltage;
    float power_mW;
    float total_mAH;
    float peekCurrent_mA;
    bool ina219_overflow = false;
#endif

// Add support files for the NEOPIXEL Status LED
#ifdef __ENABLE_NEOPIXEL__
    #include <Adafruit_NeoPixel.h>

    #define NUMPIXELS       1

    void neopixel_led(uint8_t color);

    uint8_t rgb_values[3];
    int __brightness__ = 128;  

    #define NEOPIXEL_RED    1
    #define NEOPIXEL_GREEN  2
    #define NEOPIXEL_BLUE   3
    #define NEOPIXEL_WHITE  4
    #define NEOPIXEL_ON     __brightness__
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
    Bounce2::Button open_switch = Bounce2::Button();
    Bounce2::Button close_switch = Bounce2::Button();
#endif 

// TO add EEPROM support to store settings uncomment the #define __USE_EEPROM__ in the main file
#ifdef __ENABLE_ESTOP_SWITCH__
    Bounce2::Button estop_switch = Bounce2::Button();
#endif

#endif
// SAMPLE INIT STRING
// GET FIRNWARE INFO, SET SPEED/ACCEL/BREAK, Check Position and Make sure closed
// M115 M220 S220 A0 B0 M114 M805