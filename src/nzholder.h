/**************************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 1.0.0
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/Nozzle-Magazine-Firmware
 *
 * This firmware is licensed under a GPLv3 License
 * 
 * Deltaprinter Nozzle Magazine Driver is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License version 3 as published by the 
 * Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * 
 * For more specific details see http://www.gnu.org/licenses, the Quick Guide to GPLv3. 
 * 
 * The GNU operating system has an informative GPL FAQ https://www.gnu.org/licenses/gpl-faq.html.
 * 
 **************************************************************************************************/

#ifndef nzholder_h
#define nzholder_h

  #include <Arduino.h>

  #include <DRV8871.h>    // https://github.com/dirkk1980/arduino_adafruit-drv8871
  #include <gcode.h>      // https://github.com/tinkersprojects/G-Code-Arduino-Library
  //
  // Firmware/Controler Informaiton Reported In M115
  //
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

  //
  // Just To Make Some Code More Readable
  //
  #define CLOSED  false
  #define OPEN    true
  
  //
  // Pin Configuration For Adafruit Metro Express
  //
  #ifdef ADAFRUIT_METRO_M0_EXPRESS
      #define MOTOR_IN1                 9
      #define MOTOR_IN2                 10
      #define LIMIT_CLOSE               11
      #define LIMIT_OPEN                12
      #ifdef __ENABLE_ESTOP_BUTTON__
        #define ESTOP_BUTTON            7
      #endif
      #ifdef __ENABLE_OC_BUTTON__
        #define OPEN_BUTTON             2
        #define CLOSE_BUTTON            3
      #endif
      #ifdef __ENABLE_NEOPIXEL__
        #define NEOPIXEL_PIN            40
      #endif 
      #define ACTIVITYLED               LED_BUILTIN   // LED_BUILTIN PIN_LED_13  (13u)

      // Until I figure a way to set this array programaticaly you'll have to set it yourself
      // this prevents M42 from using PIN's assigned for the Nozzle Magazine Controler itself
      #define PINCOUNT                  8
      int PinArrary[PINCOUNT] = { MOTOR_IN1, MOTOR_IN2, LIMIT_CLOSE, LIMIT_OPEN, ESTOP_BUTTON, OPEN_BUTTON, CLOSE_BUTTON, NEOPIXEL_PIN  };
  #endif

  //
  // Pin Configuration For Adafruit Trinket M0 
  // Fewer Io Pins So Can't Do Everything At Same Time
  //
  #ifdef ADAFRUIT_TRINKET_M0
      #define MOTOR_IN1               4
      #define MOTOR_IN2               5
      #define LIMIT_CLOSE             2
      #define LIMIT_OPEN              3
      #ifdef __ENABLE_ESTOP_BUTTON__
          #define ESTOP_BUTTON        0
      #endif
      #define ACTIVITYLED             LED_BUILTIN
      #ifdef __ENABLE_NEOPIXEL__
          #define NEOPIXEL_PIN        NEOPIXEL_BUILTIN
      #endif 
  #endif

  //
  // Declarations
  // Naming: How Functions Are Named
  // Standard: Camelcase
  // Gcode Callbacks: Gcodename_Camalecase 
  //
  int CheckParms();                                     // Support Function To Read Common Parms               
  void G4_Dwell();                                      // G4 S<VALUE>
  void M42_SetPinState();                                  // M42 - Set Pin State / M42 [M<0|1|2|3>] [P<pin>] S<state>
  void M111_DebugLevel();                                    // M111 - Debug Level / M111 [S<flags>]
  void M112_EmergencyStop();                                    // M112 - Emergency Stop / M112 [P<level>]
  void M114_GetCurrentPosition();                            // M114 - Get Current Position / M114 [D] [E] [R]
  void M115_FirmwareInfo();                           // M115 - Firmware Info / M115
  void M119_EndstopStates();                             // M119 - Endstop States / M119
  void M220_SetFeedratePercentage();                              // M220 - Set Feedrate Percentage / M220 S<VALUE> A<VALUE> D<VALUE>
  void M303_PIDAutoTune();                                 // M303 - PID autotune / M303 C<count> D<action> D<flag> [E<index>] S<temp> U<flag> (TBD)
  void M804_OpenNozzleMagazine();                       // M804 - Open Nozzle Magazine / M804 S<VALUE> A<VALUE> D<VALUE>
  void M805_CloseNozzleMagazine();                      // M805 - Close Nozzle Magazine / M805 S<VALUE> A<VALUE> D<VALUE>

  //
  // Variables
  // Naming: First Lower Case Characters Are Variable Type, Cammel Case Variable Name
  //
  int iSpeed = 220;               // Max Value To Not Stall @ 12V VMotor,  220 @ 9V VMotor 
  int iAcceleration = 0;          // No Acceleration Delay
  int iBrake = 0;                 // Stop Immediate
  int iDebugLevel = 0;            // L[LEVEL] Where 0=Off, 1=trace, 2=ina219 output, 3=O/C Timing
  unsigned long ulTimeout = 2000; // Default Timeout Value Of 2 Seconds
  bool bEstop = false;            // Are We ESTOPPED?
  bool bError = false;            // Are We In A Error Condition
  bool bWaiting = false;          // Are We Waiting For A Command

  DRV8871 deltaprintr_motor(
    MOTOR_IN1,
    MOTOR_IN2
  );

  //
  // How Many Commands Are Defined To Driver
  //
  #define NumberOfCommands 11

  //
  // Command Callbacks For Motor Driver
  //
  commandscallback commands[NumberOfCommands] = { 
    { "G4",   G4_Dwell                    }, // G4   S[VALUE] P[VALUE]  
    { "M42",  M42_SetPinState             }, // M43  P[VALUE] F[VALUE] S[VALUE]
    { "M111", M111_DebugLevel             }, // M111 S[VALUE]
    { "M112", M112_EmergencyStop          }, // M112 S[BOOL]
    { "M114", M114_GetCurrentPosition     }, // M114
    { "M115", M115_FirmwareInfo           }, // M115
    { "M119", M119_EndstopStates          }, // M119
    { "M220", M220_SetFeedratePercentage  }, // M220 S[VALUE] A[MILLISECONDS] B[VALUE] T[VALUE] D[VALUE]
    { "M303", M303_PIDAutoTune            }, // M303 C[COUNT] NOT IMPLEMENTED
    { "M804", M804_OpenNozzleMagazine     }, // M804
    { "M805", M805_CloseNozzleMagazine    }  // M805 
  };

  //
  // Initialize the Driver Command Structure
  //
  gcode GCode(NumberOfCommands, commands);

  //
  // To Add Current Monitoring Uncomment The #Define __ENABLE_INA219__ In The Main File
  //
  #ifdef __ENABLE_INA219__
      #include <Wire.h>
      #include <INA219_WE.h>  // https://github.com/wollewald/INA219_WE
                              // https://learn.adafruit.com/adafruit-metro-m0-express/pinouts
      #define I2C_ADDRESS 0x40
      #define MINBUSVOLTAGE 8.0 

      void ReadINA219(); // Declare The Function Since Its Now Being Used

      INA219_WE ina219 = INA219_WE(I2C_ADDRESS);

      // Define Some Global Variables.   Maybe Move Into A Structure Instead???
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

  //
  // To Add Support Files For The Neopixel Status Led Uncomment The #Define __ENABLE_NEOPIXEL__ In The Main File
  //
  #ifdef __ENABLE_NEOPIXEL__
      #include <Adafruit_NeoPixel.h>

      void NeoPixelLed(uint8_t color, uint8_t brightness); // Control NeoPixel

      int iBrightness = 200;  
      int iPulsespeed = 2000;  // Default 2000 Slow Pulse For Blue Waiting,  600 For Red Estop

      #define NUMPIXELS       1
      #define NEOPIXEL_RED    1
      #define NEOPIXEL_GREEN  2
      #define NEOPIXEL_BLUE   3
      #define NEOPIXEL_WHITE  4
      #define NEOPIXEL_AMBER  5
      #define NEOPIXEL_YELLOW 6
      #define NEOPIXEL_ON     iBrightness
      #define NEOPIXEL_OFF    0
      #define PULSEFAST       600
      #define PULSESLOW       2000

      Adafruit_NeoPixel strip(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

      // Code To Support Pulsing Neopixel
      uint8_t strip0_loop0_eff0();
      uint8_t strip0_loop0_eff1();
      uint8_t strip0_loop0();
      void strips_loop();
      void NeoPixelLed(uint8_t color, uint8_t brightness);

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

  //
  // To Add FRAM Support To Store Settings Uncomment The #Define __ENABLE_FRAM__ In The Main File
  // Why FRAM:   Because I Have Several On Hand
  // 
  #ifdef __ENABLE_FRAM__
   
  #endif

  //
  // To Add Open/Close Button Support Uncomment the #DEFINE __USE_OC_SWITCH__ In The Main File
  //
  #ifdef __ENABLE_OC_BUTTON__
      #include <Bounce2.h> // https://github.com/thomasfredericks/Bounce2

      void OpenNozzleMagazine();  // Manual Button To Open Nozzle
      void CloseNozzleMagazine(); // Manual BUtton To Close Nozzle

      // Create open/close buttons
      Bounce2::Button OpenButton = Bounce2::Button();
      Bounce2::Button ClosesButton = Bounce2::Button();
  #endif 

  //
  // To Add ESTOP Button Support Uncomment the #DEFINE __USE_ESTOP_SWITCH__ In The Main File
  //
  #ifdef __ENABLE_ESTOP_BUTTON__
      Bounce2::Button EStopButton = Bounce2::Button();
  #endif

#endif

// Sample Init String To
// Clear E-Stop, Get Firnware Info, Set Speed/Accel/Break/Timeout, Get Position
// M112 P1 M115 M220 S220 A0 B0 T2000 M114