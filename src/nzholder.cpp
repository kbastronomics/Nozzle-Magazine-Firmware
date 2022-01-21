/**************************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 1.0.0
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/Nozzle-Magazine-Firmware
 *
 * This firmware is licensed under a GPLv3 License
 **************************************************************************************************/

// Uncomment to use these features
#define __LARGE_NOZZLE__              // Uncomment for large Nozzle Magazine (20)
//#define __SMALL_NOZZLE__            // Uncomment for small Nozzle Magazine (10)
#define __ENABLE_ESTOP_SWITCH__       // Add a Physical E-STOP switch to board 
#define __ENABLE_OC_SWITCH__          // Add Physical Open/Close Switches on board
#define __ENABLE_INA219__             // Add INA219 current monitoring on board
#define __ENABLE_NEOPIXEL__           // Enables the NeoPixel in addition to the BUILTIN LED for status
                                      // Blue waiting, Green executing, Red Error, Pulsing Red ESTOP
//#define __ENABLE_EEPROM__           // Add EEPROM to store settings board

#include "nzholder.h"

//
// BEGIN Support Functions
//
#ifdef __ENABLE_NEOPIXEL__
//
// Stripe effects from https://adrianotiger.github.io/Neopixel-Effect-Generator/
//
uint8_t strip0_loop0_eff0() {
    // Strip ID: 0 - Effect: Fade - LEDS: 1
    // Steps: 500 - Delay: 1
    // Colors: 2 (0.0.255, 0.0.50)
    // Options: duration=500, every=1, 
  if(millis() - strip_0.effStart < 1 * (strip_0.effStep)) return 0x00;
  uint8_t r,g,b;
  double e;
  e = (strip_0.effStep * 1) / (double)iPulsespeed;
  ( bEstop == true) ?  r = ( e ) * 10 + 255 * ( 1.0 - e ) : r = ( e ) * 0 + 0 * ( 1.0 - e ) ;
  g = ( e ) * 0 + 0 * ( 1.0 - e );
  ( bEstop == false) ?  b = ( e ) * 10 + 255 * ( 1.0 - e ) : b = ( e ) * 0 + 0 * ( 1.0 - e ) ;
  for(uint16_t j=0;j<1;j++) {
    if((j % 1) == 0)
      strip_0.strip.setPixelColor(j, r, g, b);
    else
      strip_0.strip.setPixelColor(j, 0, 0, 0);
  }
  if(strip_0.effStep >= iPulsespeed) {strip_0.Reset(); return 0x03; }
  else strip_0.effStep++;
  return 0x01;
}

uint8_t strip0_loop0_eff1() {
    // Strip ID: 0 - Effect: Fade - LEDS: 1
    // Steps: 500 - Delay: 1
    // Colors: 2 (0.0.50, 0.0.255)
    // Options: duration=500, every=1, 
  if(millis() - strip_0.effStart < 1 * (strip_0.effStep)) return 0x00;
  uint8_t r,g,b;
  double e;
  e = (strip_0.effStep * 1) / (double)iPulsespeed;
  ( bEstop == true) ?  r = ( e ) * 255 + 10 * ( 1.0 - e ) : r = ( e ) * 0 + 0 * ( 1.0 - e ) ;
  g = ( e ) * 0 + 0 * ( 1.0 - e );
  ( bEstop == false) ?  b = ( e ) * 255 + 10 * ( 1.0 - e ) : b = ( e ) * 0 + 0 * ( 1.0 - e ) ;
  for(uint16_t j=0;j<1;j++) {
    if((j % 1) == 0)
      strip_0.strip.setPixelColor(j, r, g, b);
    else
      strip_0.strip.setPixelColor(j, 0, 0, 0);
  }
  if(strip_0.effStep >= iPulsespeed) {strip_0.Reset(); return 0x03; }
  else strip_0.effStep++;
  return 0x01;
}

uint8_t strip0_loop0() {
  uint8_t ret = 0x00;
  switch(strip0loop0.currentChild) {
    case 0: 
           ret = strip0_loop0_eff0();break;
    case 1: 
           ret = strip0_loop0_eff1();break;
  }
  if(ret & 0x02) {
    ret &= 0xfd;
    if(strip0loop0.currentChild + 1 >= strip0loop0.childs) {
      strip0loop0.currentChild = 0;
      if(++strip0loop0.currentTime >= strip0loop0.cycles) {strip0loop0.currentTime = 0; ret |= 0x02;}
    }
    else {
      strip0loop0.currentChild++;
    }
  };
  return ret;
}

void strips_loop() {
  if(strip0_loop0() & 0x01) {
    strip_0.strip.setBrightness(iBrightness);
    strip_0.strip.show();
  }
}

//
// neopixel_led(): Turn On/Off the NeoPixel for fixed colors
// used for OK/ERROR indication
//
void neopixel_led(uint8_t color, uint8_t brightness) {

strip.setBrightness(brightness);

  switch(color) {
    case(NEOPIXEL_RED):
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      break;
    case(NEOPIXEL_GREEN):
      strip.setPixelColor(0,strip.Color(0, 255, 0));
      break;  
    case(NEOPIXEL_BLUE):
      strip.setPixelColor(0,strip.Color(0, 0, 255));
      break;
    case(NEOPIXEL_WHITE):
      strip.setPixelColor(0,strip.Color(255, 255, 255));
      break;
    default:
      strip.setPixelColor(0,strip.Color(0, 0, 0));
      break;
  }
  strip.show();
}
#endif 

#ifdef __ENABLE_INA219__
//
// read_ina219:  Read the INA219 and store values into globals
//
void read_ina219() {
  // Read voltage and current from INA219.

  fShuntvoltage = ina219.getShuntVoltage_mV();
  fBusvoltage = ina219.getBusVoltage_V();
  fCurrentmA = ina219.getCurrent_mA();
  fPowermW = ina219.getBusPower();
  bIna219overflow = ina219.getOverflow();

  (bIna219overflow) ? Serial.println("Overflow! Choose higher PGAIN") :

  // record the peek current used will be reset after a M114 is issued
  (fCurrentmA > fPeekCurrentmA) ? fPeekCurrentmA = fCurrentmA :
 
  // Compute load voltage, power, and milliamp-hours.
  fLoadvoltage = fBusvoltage + (fShuntvoltage / 1000);
  fTotalmA += fCurrentmA;
  ulTotalsec += 1;
  fTotalmAH = fTotalmA / 3600.0;  

  // if we have the ISA219 in use we'll report current stats
  Serial.print("sv: "); 
  Serial.print(fShuntvoltage);
  Serial.print(" bv: ");
  Serial.print(fBusvoltage);
  Serial.print(" cmA: ");
  Serial.print(fCurrentmA);
  Serial.print(" pmA: ");
  Serial.print(fPeekCurrentmA);
  Serial.print(" lv: ");
  Serial.print(fLoadvoltage);
  Serial.print(" pmW: ");
  Serial.print(fPowermW);
  Serial.print(" tmAH: ");
  Serial.println(fTotalmAH);
}
#endif

//
// checkParms:   Checks for the A|B|S|T PARMS and stores the results in globals
//
int checkParms() {
  int _nSpeedtmp = iSpeed;
  int _nAccellerationtmp = iAcceleration;
  int _nBreaktmp = iBrake;
  int _timeouttmp = ulTimeout;
  #ifdef __ENABLE_NEOPIXEL__
    int _brightnesstmp = iBrightness;
  #endif
  int rc = 0;

  // rc=0 no parms passed
  // rc=1 error in any of the parms parms
  // rc=2 parm passed

  // if [S<percent>] is sent get the value and save it
  if (GCode.availableValue('S')) {
    iSpeed = (int) GCode.GetValue('S');
    if (iSpeed < 30 || iSpeed >= 255) {
      Serial.println(";Error: S Value out of range");
      iSpeed = _nSpeedtmp;
      Serial.println("!error");
      return 1;
    }
    rc = 2;
  }
  // if [A<Milliseconds>] is sent get the value and save it
  if (GCode.availableValue('A')) {
    iAcceleration = (int) GCode.GetValue('A');
    if (iAcceleration < 0 || iAcceleration > 1000) {
      Serial.println(";Error: A Value out of range");
      iAcceleration = _nAccellerationtmp;
      Serial.println("!error");
      return 1;
    }
    rc = 2;
  }
  // if [B<value>] is sent get the value and save it
  if (GCode.availableValue('B')) {
    iBrake = (int) GCode.GetValue('B');
    if (iBrake < 0 || iBrake > 100) {
      Serial.println(";Error: B Value out of range");
      iBrake = _nBreaktmp;
      Serial.println("!error");
      return 1;
    }
    rc = 2;
  }

  // if [T<value>] is sent get the value and save it
  if (GCode.availableValue('T')) {
    ulTimeout = (int) GCode.GetValue('T');
    if (ulTimeout < 0 || ulTimeout > 10000) {
      Serial.println(";Error: T Value out of range");
      ulTimeout = _timeouttmp;
      Serial.println("!error");
      return 1;
    }
  }

  #ifdef __ENABLE_NEOPIXEL__
  // if [D<value>] is sent get the value and save it
  if (GCode.availableValue('D')) {
    iBrightness = (int) GCode.GetValue('D');
    if (iBrightness < 0 || iBrightness > 255) {
      Serial.println(";Error: D Value out of range");
      iBrightness = _brightnesstmp;
      Serial.println("!error");
      return 1;
    }
    rc = 2;
  }
  #endif 

  return rc;
}
//
// END Support Functions
//


//
// BEGIN callback functions,   all implemented as blocking calls
//

//
// G4_pause:  Delays for value S in seconds or P in Milliseconds 
// Has no real effect due to blocking nature of calls
//
void G4_pause() {
  int delayvaule = 0;

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON);
  #endif

  Serial.println(">G4");
  // if [S<VALUE<] this is in seconds so multiple by 1000 
  if (GCode.availableValue('S')) {
    delayvaule = (unsigned long) GCode.GetValue('S') * 1000;
  }

  // if [P<VALUE<] this is in milliseconds so use as-is
  if (GCode.availableValue('P')) {
    delayvaule = (unsigned long) GCode.GetValue('P');
  }

  delay(delayvaule); // Now wait for that time period
  Serial.println("!ok");

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF);
  #endif
}

//
// M111_debug:  Sets Debug Flags P<MODULE> L<LEVEL>
// P[MODULE] is the numeric value of the call the be debuged,  ignored is L2
// L[LEVEL] 0=DISABLE, 1=MODULE, 2=ALL
//
void M111_debug() {

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.println(">M111");

  // if [P<VALUE>] this is the module #,  ie. M804 would be 804
  iDebugmodule = (int) (GCode.availableValue('P')) ?  GCode.GetValue('P') : 0 ;

  // if [L<VALUE<] this is debug level (L0=OFF, L1=MODULE ONLY, L2=ALL) 
  iDebuglevel = (int) (GCode.availableValue('L')) ? GCode.GetValue('L') : 0 ;

  Serial.print(";Debug p");
  Serial.print(iDebugmodule);
  Serial.print(" l");
  Serial.println(iDebuglevel);
  Serial.println("!ok");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M112_estop:  Cause an immediate stop to motion
//
void M112_estop() {

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.println(">M112");
  
  // if [P<value>] If 1 clear ESTOP and we're done
  if (GCode.availableValue('P')) {
    int value = (int) GCode.GetValue('P');
    if (value == 1 ) {
      bEstop = false;
      Serial.println("E-STOP Cleared");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_BLUE, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        iPulsespeed = 2000;
      #endif
      digitalWrite(ACTIVITYLED, LOW);

      return;
    }
  }

  deltaprintr_motor.breakdown(0, 0);
  Serial.println("E-STOP");
  Serial.println("!ok");
  bEstop = true;
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS RED DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    iPulsespeed = 600;
  #endif
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M114_reportPostion:  Reports current postion information
//
void M114_reportPostion() {
  String state = "";
  int istate = 0;

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);
  Serial.println(">M114");

  //  get current state of limit switch's
  if (digitalRead(LIMIT_OPEN) == LOW) {
    state = "open";
  }

  if (digitalRead(LIMIT_CLOSE) == LOW) {
    state = "close";
  }

  // if both limits are open we're stuck between them, allrt user
  if (digitalRead(LIMIT_OPEN) == HIGH && digitalRead(LIMIT_CLOSE) == HIGH) {
    state = "Magazine Slide Stuck between Open/Close";
    bError = true;
    istate = 1;
  }

  // if both limits are closed we have a issue, alert user
  if (digitalRead(LIMIT_OPEN) == LOW && digitalRead(LIMIT_CLOSE) == LOW) {
    state = "Magazine Slide Limit Switch Possibly Shorted";
    bError = true;
    istate = 2;
  }

  #ifdef __ENABLE_INA219__
  if ( iDebuglevel == 2) {
    read_ina219();
  }
  #endif

  if ( iDebuglevel == 5) { // print vars
    Serial.print(";error:");
    Serial.print(bError);
    Serial.print(" estop:");
    Serial.print(bEstop);
    Serial.print(" sleep:");
    Serial.println(bWaiting);
  }
  
  // Send final message
  Serial.print("p:");
  Serial.println(state);
  (istate > 0) ? Serial.println("!error") : Serial.println("!ok");

  #ifdef __ENABLE_NEOPIXEL__
    ( bError == false) ? neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF) : neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON) ; 
  #endif
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M115_reportFirmware:   Reports firmware information
//
void M115_reportFirmware() {
  char buffer[512];

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.println(">M115");

  sprintf(buffer, "Firmware_Name: %s Firmware_Version: %s (Github) Source_Code_URl: %s Machine_Type: %s\
  Nozzle_Count: %d Control_Board: %s Motor_Driver: %s Current_Monitor: %s UUID: %s", FIRMWARE_NAME, FIRMWARE_VERSION, GITHUB_URL, MACHINE_TYPE, \
   NOZZLE_COUNT, CONTROL_BOARD, MOTOR_DRIVER, CURRENT_MONITOR, UUID);
  
  Serial.println(buffer);
  Serial.println("!ok");

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif 
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M119_endStopState: report the current state of the endstops
//
void M119_endStopState() {
  String n_min = "open", n_max = "open";

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);
  Serial.println(">M119");

  n_max = (digitalRead(LIMIT_OPEN) == LOW) ? "triggered" : "open" ;
  n_min = (digitalRead(LIMIT_CLOSE) == LOW) ? "triggered" : "open" ;

  Serial.println(";Reporting Endstop State");
  Serial.print("n_min:");
  Serial.print(n_min); // n_min is closed position
  Serial.print(" n_max:");
  Serial.println(n_max); // n_max is open position

  if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
    Serial.println(";Magazine Slide Caught between Open/Close");
    Serial.println("!error");
      bError = true;
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
    return;
  }

  Serial.println("!ok");

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF);
  #endif

  digitalWrite(ACTIVITYLED, LOW);
  return;
}

//
// M220_setFeedrate: Sets the Speed, Acceleration and Brake values 
//
void M220_setFeedrate() {
  
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH); 

  if (checkParms() == 1) { // ERROR So do nothing
    return;
  }

  // echo gcode sent
  Serial.println(">M220");
  Serial.print("s:");
  Serial.print(iSpeed);
  Serial.print(" a:");
  Serial.print(iAcceleration);
  Serial.print(" b:");
  Serial.print(iBrake);
  Serial.print(" t:");
  Serial.print(ulTimeout);
  #ifdef __ENABLE_NEOPIXEL__
    Serial.print(" d:");
    Serial.println(iBrightness);
  #endif 

  Serial.println("!ok");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M303_autotune: Perform autotune to find the best speed and acceleration values
//
void M303_autotune() {
  int count = 5; // default number of cycles to find the optiman speed/acceleration values
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif  
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.println(">M303");

  // if [C<value>] is sent get the value and save it
  if (GCode.availableValue('C')) {
    count = (int) GCode.GetValue('C');
    if (count <= 0 || count > 1000) {
      Serial.println("Error: C Value out of range");
      count = 5;
      Serial.println("!ok");
      return;
    }
  }

  // INSERT AUTOTUNING HERE
  for (int i = 0; i <= count; i++) {
    #ifdef __ENABLE_ESTOP_SWITCH__
    if (Estopswitch.pressed()) {
      deltaprintr_motor.breakdown(0, 0);
      Serial.println(";Emergincy Stop Triggered");
      Serial.println("!estop");
    }
    #endif
    GCode.comment('c', (double) count);
  }
  Serial.println("!ok");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif  
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M804_openNozzlemagazine:  Open the magazine or report open if already open
//
void M804_openNozzleMagazine() {
    unsigned long time1 = 0, time2 = 0;

    if (bEstop == true) {
      Serial.println("E-STOP");
      Serial.println("!error");
      return;
    }

    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine
    Serial.println(">M804"); // echo the command

    // If the LIMIT_OPEN is already set just exit since we already are open
    if (digitalRead(LIMIT_OPEN) == LOW) {
      if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) {
        Serial.println(";Debug:407 digitalRead(LIMIT_OPEN) == LOW ");
        Serial.print(";Debug:408 S: ");
        Serial.print(iSpeed);
        Serial.print(" A: ");
        Serial.println(iAcceleration);
      }
      Serial.println("n:open");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // Check for any passed parms and if get a rc=1 we had an invalid parm so exit.
    if (checkParms() == 1) {
      if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) {
        Serial.println(";Debug:422 checkParms()");
      }
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) {
        Serial.println(";Debug:431 digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH");
      }
      Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {
        #ifdef __ENABLE_INA219__
        if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) {
          read_ina219();
        }
        #endif
      }
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.println("!error");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // No that the checks are done we can actually move the motor
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) // if debuging print some information
      {
        Serial.println(";Debug:453 digitalRead(LIMIT_CLOSE) == LOW ");
        Serial.print(";Debug:454 s:");
        Serial.print(iSpeed);
        Serial.print(" s:");
        Serial.println(iAcceleration);
      }

      time1 = millis();
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);

      if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) {
        Serial.print(";Debug:464 s:");
        Serial.print(deltaprintr_motor.currentSpeed());
        Serial.print(" a:");
        Serial.print(iAcceleration);
        Serial.print(" d:");
        Serial.println(deltaprintr_motor.currentDirection());
      }
      // Wait loop for limit to be hit
      while (digitalRead(LIMIT_OPEN) == HIGH) // This is the main loop that waits for the LIMIT_OPEN switch to close 
      {
        #ifdef __ENABLE_INA219__
          if ( iDebuglevel >= 3) {
            read_ina219();
          }
        #endif
        time2 = millis(); // grap the current mS
        if (GCode.availableValue('M')) // check if a M112 is sent and if so ESTOP  
        {
          int code = (int) GCode.GetValue('M');
          if (code == 112) { // GOT ESTOP COMMAND
            Serial.println(";Received M112 to E-STOP");
            deltaprintr_motor.breakdown(0, 0);
            Serial.println("!error");
             bEstop = true;
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
            digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
            return; // Since we E-STOP we just exit              }
          }
        }
          #ifdef __ENABLE_ESTOP_SWITCH__ // if we're using the ESTOP SWITCH check the ping and stop then return
          if (Estopswitch.pressed()) {
            deltaprintr_motor.breakdown(0, 0);
            Serial.println(";Emergincy Stop Triggered");
            Serial.println("!estop");
            bEstop = true;
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
            return;
          }
          #endif
          if ((time2 - time1) >= ulTimeout) // check if we've timed out and if so set error and return
          {
            Serial.println(";Timed out opening Nozzle");
            Serial.println("!error");
            bError = true;
            deltaprintr_motor.breakdown(0, 0);
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
            digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
            return;
          }
      } // ends that main loop for the LIMIT_OPEN
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
        time2 = millis();

        if ((iDebugmodule == 804 && iDebuglevel == 1) || iDebuglevel == 2) {
          Serial.print(";Debug:512 Move took (mS):");
          Serial.println(time2 - time1);
        }

      }
      // now let user know we're open and things are OK.
      #ifdef __ENABLE_INA219__
        fPeekCurrentmA = 0;
      #endif
      Serial.println("n:open");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
       bEstop = false;
      bError = false;
}

//
// M805_closeNozzlemagazine: Close the magazine or report open if already closed
//
 void M805_closeNozzleMagazine() {
      unsigned long time1 = 0, time2 = 0;

      #ifdef __ENABLE_ESTOP_SWITCH__
      if (bEstop == true) {
        Serial.println("E-STOP");
        Serial.println("!error");
        return;
      } 
      #endif 

      digitalWrite(ACTIVITYLED, HIGH);
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      Serial.println(">M805");

      // If the LIMIT_CLOSE is already set just exit since we already are closed
      if (digitalRead(LIMIT_CLOSE) == LOW) {
        if ((iDebugmodule == 805 && iDebuglevel == 1) || iDebuglevel == 2) {
          Serial.println(";Debug:546 digitalRead(LIMIT_CLOSE) == LOW ");
          Serial.print(";Debug:547 s:");
          Serial.print(iSpeed);
          Serial.print(" a:");
          Serial.println(iAcceleration);
        }
        Serial.println("n:close");
        Serial.println("!ok");
        #ifdef __ENABLE_NEOPIXEL__
          neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        #endif
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // Check for any passed parms
      if (int i = checkParms() == 1) {
        if ((iDebugmodule == 805 && iDebuglevel == 1) || iDebuglevel == 2) {
          Serial.print(";Debug:561 checkParms() state=");
          Serial.println(i);
        }
        #ifdef __ENABLE_NEOPIXEL__
          neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        #endif
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // If Magazine is caught between Open/CLose force it to close
      if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
        if ((iDebugmodule == 805 && iDebuglevel == 1) || iDebuglevel == 2) {
          Serial.println(";Debug:560 Limits Error: Magazine not at either limit");
        }
        Serial.println(";Limits Error: Magazine not at either limit");
        deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);
        while (digitalRead(LIMIT_OPEN) == HIGH) {} // wait for LIMIT_OPEN switch to go low and soon as it does we 
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
        Serial.println("!error");
        bError = true;
        #ifdef __ENABLE_NEOPIXEL__
          neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        #endif
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // Now that the checks are done we can actually move the motor
      if (digitalRead(LIMIT_OPEN) == LOW) {
        if ((iDebugmodule == 805 && iDebuglevel == 1) || iDebuglevel == 2) {
          Serial.println(";Debug:585 Not at Limit Open, Moving to Open Position");
          Serial.print(";Debug:586 s:");
          Serial.print(iSpeed);
          Serial.print(" a:");
          Serial.println(iAcceleration);
        }
        time1 = millis();
        deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
        if ((iDebugmodule == 805 && iDebuglevel == 1) || iDebuglevel == 2) {
          Serial.print(";Debug:594 s:");
          Serial.print(deltaprintr_motor.currentSpeed());
          Serial.print(" a:");
          Serial.print(iAcceleration);
          Serial.print(" d:");
          Serial.println(deltaprintr_motor.currentDirection());
        }
        // Wait loop for limit to be hit
        while (digitalRead(LIMIT_CLOSE) == HIGH) {
          #ifdef __ENABLE_INA219__
          if ( iDebuglevel >= 3) {
          read_ina219();
          }
          time2 = millis();
          #endif
          // timetmp = time2-time1;
          #ifdef __ENABLE_ESTOP_SWITCH__
          if (Estopswitch.pressed()) {
            deltaprintr_motor.breakdown(0, 0);
            Serial.println(";Emergi1ncy Stop Triggered");
            Serial.println("!error");
            bEstop = true;
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
          }
          #endif
          if ((time2 - time1) >= ulTimeout) {
            Serial.println(";Timed out opening Nozzle");
            Serial.println("!error");
            bError = false;
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
            deltaprintr_motor.breakdown(0, 0);
            return;
          }
        } // wait for LIMIT_OPEN switch to go low and soon as it does we 
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      }
      time2 = millis();
      if ((iDebugmodule == 805 && iDebuglevel == 1) || iDebuglevel == 2) {
        Serial.print(";Debug:618 Move took (mS):");
        Serial.println(time2 - time1);
      }

      // now let user know we're open and things are OK.
      #ifdef __ENABLE_INA219__
        fPeekCurrentmA = 0;
      #endif 
      Serial.println("n:closed");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      bEstop = false;
      bError = false;

    }
 
//
// END callback functions,   all implemented as blocking calls
//

#ifdef __ENABLE_OC_SWITCH__
//
// openNozzle:  Manuualy open the Nozzle via push button on controller
//
void openNozzleMagazine() {

    Serial.println(";Open Button Pressed"); // echo the command
    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine

    if (bEstop == true) {
      Serial.println("E-STOP");
      Serial.println("!error");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If the LIMIT_OPEN is already set just exit since we already are open
    if (digitalRead(LIMIT_OPEN) == LOW) {
      Serial.println(";Already Open");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.println(";error");
      bError = false;
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_RED, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif  
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);

      while (digitalRead(LIMIT_OPEN) == HIGH) { 
        #ifdef __ENABLE_INA219__
        if ( iDebuglevel >= 3) {
          read_ina219();
        }
        #endif 
      } // ends that main loop for the LIMIT_OPEN
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
    }
      // now let user know we're open and things are OK.
      #ifdef __ENABLE_INA219__      
        fPeekCurrentmA = 0;
      #endif 
      Serial.println("n:open");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      bEstop = false;
      bError = false;
      digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
}

//
// closeNozzle:  Manuualy close the Nozzle via push button on controller
//
void closeNozzleMagazine() {

    Serial.println(";Close Button Pressed"); // echo the command
    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine

    if (bEstop == true) {
      Serial.println("E-STOP");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
        digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If the LIMIT_CLOSE is already closed just exit since we already are closed
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      Serial.println(";Already Closed");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.println(";error");
      bError = false;
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_RED, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif  
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
    if (digitalRead(LIMIT_OPEN) == LOW) {
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) { 
        #ifdef __ENABLE_INA219__
        if ( iDebuglevel >= 3) {
          read_ina219();
        }
        #endif
      } 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
    }
      // Now let user know we're open and things are OK.
      #ifdef __ENABLE_INA219__
        fPeekCurrentmA = 0;
      #endif
      Serial.println("n:closed");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      bEstop = false;
      bError = false;
      digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
}         
#endif     

void setup() {

  // Call to start GCODE Object
  GCode.begin(115200, ">"); //responce => ok, rs or !!
  while(!Serial.availableForWrite()) {}; // wait for serial to become available
  
  Serial.println("Nozzle Magazine Controller (c) 2022 KBAstronomics");

  // Setup our INPUT/OUTPUTS
  pinMode(ACTIVITYLED, OUTPUT);
  pinMode(LIMIT_CLOSE, INPUT_PULLUP);
  pinMode(LIMIT_OPEN, INPUT_PULLUP);

  #ifdef __ENABLE_OC_SWITCH__
    // CREATE BUTTONS
    Openswitch.attach( OPEN_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    Closeswitch.attach( CLOSE_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    
    // DEBOUNCE INTERVAL IN MILLISECONDS
    Openswitch.interval(5);
    Closeswitch.interval(5); 
  
    // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
    Openswitch.setPressedState(LOW);
    Closeswitch.setPressedState(LOW); 
  #endif 
 
  // If the ESTOP is defined create the button with a 5ms debounce and active low 
  #ifdef __ENABLE_ESTOP_SWITCH__
    Estopswitch.attach( ESTOP_SWITCH ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    Estopswitch.interval(5);
    Estopswitch.setPressedState(LOW);
    bEstop = false;
  #endif

  // If the INA219 is defined set it up
  #ifdef __ENABLE_INA219__
    Wire.begin();
    if( !ina219.init() ) {
      Serial.println("INA219 not connected!");
    }

  /* Set ADC Mode for Bus and ShuntVoltage
    * Mode *            * Res / Samples *       * Conversion Time *
    BIT_MODE_9        9 Bit Resolution             84 µs
    BIT_MODE_10       10 Bit Resolution            148 µs  
    BIT_MODE_11       11 Bit Resolution            276 µs
    BIT_MODE_12       12 Bit Resolution            532 µs  (DEFAULT)
    SAMPLE_MODE_2     Mean Value 2 samples         1.06 ms
    SAMPLE_MODE_4     Mean Value 4 samples         2.13 ms
    SAMPLE_MODE_8     Mean Value 8 samples         4.26 ms
    SAMPLE_MODE_16    Mean Value 16 samples        8.51 ms     
    SAMPLE_MODE_32    Mean Value 32 samples        17.02 ms
    SAMPLE_MODE_64    Mean Value 64 samples        34.05 ms
    SAMPLE_MODE_128   Mean Value 128 samples       68.10 ms
    */
    ina219.setADCMode(BIT_MODE_12); // choose mode and uncomment for change of default
    
    /* Set measure mode
    POWER_DOWN - INA219 switched off
    Triggered  - measurement on demand
    ADC_OFF    - Analog/Digital Converter switched off
    CONTINUOUS  - Continuous measurements (DEFAULT)
    */
    ina219.setMeasureMode(CONTINUOUS); // Triggered measurements for this example
    
    /* Set PGain
    * Gain *  * Shunt Voltage Range *   * Max Current *
    PG_40       40 mV                    0.4 A
    PG_80       80 mV                    0.8 A
    PG_160      160 mV                   1.6 A
    PG_320      320 mV                   3.2 A (DEFAULT)
    */
    ina219.setPGain(PG_80); // choose gain and uncomment for change of default
    
    /* Set Bus Voltage Range
    BRNG_16   -> 16 V
    BRNG_32   -> 32 V (DEFAULT)
    */
    ina219.setBusRange(BRNG_16); // choose range and uncomment for change of default

    /* If the current values delivered by the INA219 differ by a constant factor
      from values obtained with calibrated equipment you can define a correction factor.
      Correction factor = current delivered from calibrated equipment / current delivered by INA219
    */
    // ina219.setCorrectionFactor(0.98); // insert your correction factor if necessary
  #endif

  #ifdef __ENABLE_NEOPIXEL__
    pinMode(NEOPIXEL_PIN, OUTPUT);
    digitalWrite(NEOPIXEL_PIN, LOW);
    strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.setBrightness(50);
    strip.show();  // Turn OFF all pixels ASAP
    neopixel_led(NEOPIXEL_BLUE, NEOPIXEL_ON); // Now Show a BLue Pixel 
    strip_0.strip.begin();
  #endif  
}

void loop() {
  // see if there is a GCODE command available and if not we're waiting  
  (GCode.available()) ? bWaiting = false : bWaiting = true ; 

  // if ESTOP is defined get button and if pressed stop motor
  #ifdef __ENABLE_ESTOP_SWITCH__
    Estopswitch.update();
    if (Estopswitch.pressed()) {
      M112_estop();
    }
    if ( Estopswitch.released()) {
      bEstop = false;
      Serial.println("E-STOP Cleared");
      Serial.println("!ok");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        iPulsespeed = 2000;
      #endif
      digitalWrite(ACTIVITYLED, LOW);
    }
  #endif

  #ifdef __ENABLE_OC_SWITCH__
  // get button info
    Openswitch.update();
    Closeswitch.update();
  
  // if the open button is pressed call openNozzleMagazine
    if ( Openswitch.pressed() ) {
      openNozzleMagazine();
    }

  // if the close button is pressed call closeNozzleMagazine
    if (Closeswitch.pressed()) {
      closeNozzleMagazine();
    }
  #endif 

  // check several triggers for if the status led should be BLUE(OK) or RED (ERROR CONDITION)
  #ifdef __ENABLE_NEOPIXEL__
    if ( bEstop == false && bError == false && bWaiting == false ) {
      neopixel_led(NEOPIXEL_BLUE, NEOPIXEL_ON);
    } 
    // else {
    //   neopixel_led(NEOPIXEL_RED, NEOPIXEL_OFF);
    // }

    // this should only be called after a timeout value
    if ( bWaiting == true ) { 
      strips_loop();
    }
  #endif 
}