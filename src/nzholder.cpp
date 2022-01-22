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
    case(NEOPIXEL_AMBER):
      strip.setPixelColor(0,strip.Color(255,126,0));
      break;
    case(NEOPIXEL_YELLOW):
      strip.setPixelColor(0,strip.Color(250,250,55)); 
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
  fTotalmAH = fTotalmA / ulTotalsec;  

  Serial.printf(";Line %i: sv(%3.2f) bv(%3.2f) cmA(%3.2f) ",__LINE__,fShuntvoltage,fBusvoltage,fCurrentmA);  
  Serial.printf("pmA(%3.2f) lv(%3.2f) pmW(%3.2f) maH(%3.2f)\n",fPeekCurrentmA,fLoadvoltage,fPowermW,fTotalmAH);  
 
  // Serial.print(";sv: "); 
  // Serial.print(fShuntvoltage);
  // Serial.print(" bv: ");
  // Serial.print(fBusvoltage);
  // Serial.print(" cmA: ");
  // Serial.print(fCurrentmA);
  // Serial.print(" pmA: ");
  // Serial.print(fPeekCurrentmA);
  // Serial.print(" lv: ");
  // Serial.print(fLoadvoltage);
  // Serial.print(" pmW: ");
  // Serial.print(fPowermW);
  // Serial.print(" tmAH: ");
  // Serial.println(fTotalmAH);
}
#endif

void T100_test() {
bool bTest=false;
int iTest=199;
unsigned long ulTest=1919191919;
double dTest=3.14159;
float fTest=182282.3972;
String sTest="This is a String";
char cTest[80]="This is a char array";

Serial.printf("T100 printf Tests\n");
Serial.printf("Bool %s\n",bTest?"TRUE":"FALSE");
Serial.printf("INT %i\n",iTest);
Serial.printf("ULONG %u\n",ulTest);
Serial.printf("FAIL:DOUBLE %d\n",dTest);
Serial.printf("FLOAT %f\n",fTest);
Serial.printf("FAIL:STRING %s\n",sTest);
Serial.printf("CHAR %s\n",cTest);

}

//
// checkParms:   Checks for the S,A,B,T,D PARMS and stores the results in globals
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

  // if [S<VALUE>] is sent get the value and save it
  if (GCode.availableValue('S')) {
    iSpeed = (int) GCode.GetValue('S');
    if (iSpeed < 0 || iSpeed >= 255) {
      Serial.printf(";L%iL Error: S Value(%i) out of range\n",__LINE__,iSpeed);
      iSpeed = _nSpeedtmp;
      Serial.println("error");
      return 1;
    }
    rc = 2;
  }
  // if [A<Milliseconds>] is sent get the value and save it
  if (GCode.availableValue('A')) {
    iAcceleration = (int) GCode.GetValue('A');
    if (iAcceleration < 0 || iAcceleration > 1000) {
      Serial.printf(";Line %i: Error: A Value(%i) out of range\n",__LINE__,iAcceleration);
      iAcceleration = _nAccellerationtmp;
      Serial.printf("error\n");
      return 1;
    }
    rc = 2;
  }
  // if [B<value>] is sent get the value and save it
  if (GCode.availableValue('B')) {
    iBrake = (int) GCode.GetValue('B');
    if (iBrake < 0 || iBrake > 100) {
      Serial.printf(";Line %i: Error: B Value(%i) out of range\n",__LINE__,iBrake);
      iBrake = _nBreaktmp;
      Serial.printf("error\n");
      return 1;
    }
    rc = 2;
  }

  // if [T<value>] is sent get the value and save it
  if (GCode.availableValue('T')) {
    ulTimeout = (int) GCode.GetValue('T');
    if (ulTimeout < 0 || ulTimeout > 10000) {
      Serial.printf(";Line %i: Error: T Value(%u) out of range\n",__LINE__,ulTimeout);
      ulTimeout = _timeouttmp;
      Serial.printf("error\n");
      return 1;
    }
  }

  #ifdef __ENABLE_NEOPIXEL__
  // if [D<value>] is sent get the value and save it
  if (GCode.availableValue('D')) {
    iBrightness = (int) GCode.GetValue('D');
    if (iBrightness < 0 || iBrightness > 255) {
      Serial.printf(";Line %i: Error: D Value(%i) out of range\n",__LINE__,iBrightness);
      iBrightness = _brightnesstmp;
      Serial.printf("error\n");
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
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON);
  #endif

  Serial.printf("G4\n");
  // if [S<VALUE<] this is in seconds so multiple by 1000 
  if (GCode.availableValue('S')) {
    delayvaule = (unsigned long) GCode.GetValue('S') * 1000;
  }

  // if [P<VALUE<] this is in milliseconds so use as-is
  if (GCode.availableValue('P')) {
    delayvaule = (unsigned long) GCode.GetValue('P');
  }

  delay(delayvaule); // Now wait for that time period
  Serial.printf("ok\n");

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF);
  #endif
}

//
// M42 - Set Pin State / M42 [F<0|1|2|3>] [P<pin>] S<state>
// [F<0|1|2|3>] Set the pin mode.
//    F0: INPUT
//    F1: OUTPUT
//    F2: INPUT_PULLUP
//    F3: INPUT_PULLDOWN
// [P<pin>]	A digital pin number (even for analog pins) to write to. (LED_PIN if omitted)
// S<state>	The state to set. PWM pins may be set from 0-255.
void M42_pinstate() {   
  int iPinMode=0;
  int iPinState=0;
  int iPin=0;
  int iResult=0;

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.printf("M42\n");

  // if [M<VALUE>] this is the module #,  ie. M0,M1,M2,M3
  if (GCode.availableValue('F')) {
    iPinMode = (int) GCode.GetValue('F');
    if ( iPinMode < 0 || iPinMode > 3 ) {
      Serial.printf(";Line %i: Error: F(%i) Value out of range\n",__LINE__,iPinMode);
      Serial.printf("error\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // turn off the neopixel,  we're done
      #endif
      if( iPin != ACTIVITYLED) // doing this if your trying to control the LED_BUILTIN
        digitalWrite(ACTIVITYLED, LOW); 
      return;
    }
  }
  
  if (GCode.availableValue('P')) {
    iPin = (int) GCode.GetValue('P');
    if (iPin < 0 || iPin >= 50) {
      Serial.printf(";Line %i: Error: P(%i) Value out of range\n",__LINE__,iPin);
      Serial.printf("error\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // turn off the neopixel,  we're done
      #endif
      if( iPin != 13)  // doing this if your trying to control the LED_BUILTIN
        digitalWrite(ACTIVITYLED, LOW);
      return;
    }
  }

  if (GCode.availableValue('S')) {
    iPinState = (int) GCode.GetValue('S');
    if (iPinState < 0 || iPinState >= 255) {
      Serial.printf(";Line %i: Error: S(%i) Value out of range",__LINE__,iPinState);
      Serial.println("error");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // turn off the neopixel,  we're done
      #endif
      if( iPin != 13) // doing this if your trying to control the LED_BUILTIN
        digitalWrite(ACTIVITYLED, LOW);
      return;
    }
  } 

  switch(iPinMode) {
    case 0:
      pinMode(iPin,INPUT);
      iResult = digitalRead(iPin);
      break;
    case 1:
      pinMode(iPin,OUTPUT);
      digitalWrite(iPin,iPinState);
      // todo:  account for analog outputs
      break;
    case 2:
      pinMode(iPin,INPUT_PULLUP);
      iResult = digitalRead(iPin);
      break;
    case 3:
      pinMode(iPin,INPUT_PULLDOWN);
      iResult = digitalRead(iPin);
      break;
    default:
      Serial.printf(";Line %i: Invalid PINMODE Specified\n",__LINE__);
      break;
  }
  
  if ( iPinMode != 1 ) {
    Serial.printf("P:%i R:%i\n",iPin,iResult);
  } else {
    Serial.printf("P:%i W:%i\n",iPin,iPinState);
  }

  Serial.printf("ok\n");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  if( iPin != 13) // doing this if your trying to control the LED_BUILTIN
    digitalWrite(ACTIVITYLED, LOW);
}

//
// M111_debug:  Sets Debug Flags P<MODULE> L<LEVEL>
// L[LEVEL] 0=DISABLE, 1=DEBUG
//
void M111_debug() {

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.printf("M111\n");

  // if [L<VALUE<] this is debug level (L0=OFF, L1=MODULE ONLY, L2=ALL) 
  iDebuglevel = (int) (GCode.availableValue('L')) ? GCode.GetValue('L') : 0 ;

  Serial.printf(";Debug L:%i\n",iDebuglevel);
  Serial.printf("ok\n");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
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

  Serial.printf("M112\n");
  
  // if [P<value>] If 1 clear ESTOP and we're done
  if (GCode.availableValue('P')) {
    int value = (int) GCode.GetValue('P');
    if (value == 1 ) {
      bEstop = false;
      Serial.printf("E-STOP Cleared\n");
      Serial.printf("ok\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_BLUE, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        iPulsespeed = 2000;
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }
  }

  deltaprintr_motor.breakdown(0, 0);
  Serial.printf("E-STOP\n");
  Serial.printf("ok\n");
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
  String sState = "";
  int istate = 0;

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);
  Serial.printf("M114\n");

  //  get current state of limit switch's
  if (digitalRead(LIMIT_OPEN) == LOW) {
    sState = "open";
  }

  if (digitalRead(LIMIT_CLOSE) == LOW) {
    sState = "close";
  }

  // if both limits are open we're stuck between them, allrt user
  if (digitalRead(LIMIT_OPEN) == HIGH && digitalRead(LIMIT_CLOSE) == HIGH) {
    sState = ";Magazine slide jammed";
    bError = true;
    istate = 1;
  }

  // if both limits are closed we have a issue, alert user
  if (digitalRead(LIMIT_OPEN) == LOW && digitalRead(LIMIT_CLOSE) == LOW) {
    sState = ";Magazine slide limit switch possibly shorted";
    bError = true;
    istate = 2;
  }

  #ifdef __ENABLE_INA219__
  if ( iDebuglevel == 1 ) {
    read_ina219();
  }
  #endif

  if ( iDebuglevel == 1 ) { // print vars
    Serial.printf(";Debug Line %i: error:%s estop:%s sleep:%s\n",__LINE__,bError?"true":"false",bEstop?"true":"false",bWaiting?"true":"false");
  }
  
  // Send final message printf can't use Strings 
  Serial.print("p:");
  Serial.println(sState);
  (istate > 0) ? Serial.printf("error\n") : Serial.printf("ok\n");

  #ifdef __ENABLE_NEOPIXEL__
    ( bError == false) ? neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF) : neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON) ; 
  #endif
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M115_reportFirmware:   Reports firmware information
//
void M115_reportFirmware() {
  char buffer[512];

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.printf("M115\n");

  // weird ardiuno artifact that a Serial.printf wil not work on this string but a sprintf will :(
  sprintf(buffer, "Firmware_Name: %s Firmware_Version: %s (Github) Source_Code_URl: %s Machine_Type: %s\
  Nozzle_Count: %d Control_Board: %s Motor_Driver: %s Current_Monitor: %s UUID: %s", FIRMWARE_NAME, FIRMWARE_VERSION, GITHUB_URL, MACHINE_TYPE, \
   NOZZLE_COUNT, CONTROL_BOARD, MOTOR_DRIVER, CURRENT_MONITOR, UUID);
  
  Serial.println(buffer);
  Serial.printf("ok\n");

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif 
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M119_endStopState: report the current state of the endstops
//
void M119_endStopState() {
  bool n_min=false;
  bool n_max=false;

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH);
  Serial.printf("M119\n");

  n_max = digitalRead(LIMIT_OPEN);
  n_min = digitalRead(LIMIT_CLOSE);

  if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
    Serial.printf(";Line %i: Magazine slide jammed\n",__LINE__);
    Serial.printf("error\n");
    bError = true;
    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, LOW);
    return;
  }

  Serial.printf("n_min:%s n_max:%s\n",n_min?"TRIGGERED":"open",n_max?"TRIGGERED":"open");
  Serial.printf("ok\n");

  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF);
  #endif
  digitalWrite(ACTIVITYLED, LOW);
  return;
}

//
// M220_setFeedrate: Sets the Speed, Acceleration and Brake values 
//
void M220_setFeedrate() {
  
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, HIGH); 

  if (checkParms() == 1) { // ERROR So do nothing
    return;
  }

  // echo gcode sent
  Serial.printf("M220\n");
  Serial.printf("S:%i A:%i B:%i T:%u D:%i\n",iSpeed,iAcceleration,iBrake,ulTimeout,iBrightness);

  Serial.printf("ok\n");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M303_autotune: Perform autotune to find the best speed and acceleration values
//
void M303_autotune() {
  int count = 5; // default number of cycles to find the optiman speed/acceleration values
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif  
  digitalWrite(ACTIVITYLED, HIGH);

  Serial.println("M303");

  // if [C<value>] is sent get the value and save it
  if (GCode.availableValue('C')) {
    count = (int) GCode.GetValue('C');
    if (count <= 0 || count > 1000) {
      Serial.printf(";Line %i: Error: C(%i) Value out of range\n",__LINE__,count);
      Serial.printf("ok\n");
      return;
    }
  }

  // INSERT AUTOTUNING HERE
  for (int i = 0; i <= count; i++) {
    #ifdef __ENABLE_ESTOP_SWITCH__
    if (Estopswitch.pressed()) {
      deltaprintr_motor.breakdown(0, 0);
      Serial.printf(";Line %i: Emergincy Stop Triggered\n",__LINE__);
      Serial.printf("!estop\n");
    }
    #endif
    GCode.comment('c', (double) count);
  }
  Serial.printf("ok\n");
  #ifdef __ENABLE_NEOPIXEL__
    neopixel_led(NEOPIXEL_AMBER, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
  #endif  
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M804_openNozzlemagazine:  Open the magazine or report open if already open
//
void M804_openNozzleMagazine() {
    unsigned long time1 = 0, time2 = 0;

    if (bEstop == true) {
      Serial.printf("E-STOP\n");
      Serial.printf("error\n");
      return;
    }
  
    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine
    Serial.printf("M804\n"); // echo the command

    // If the LIMIT_OPEN is already set just exit since we already are open
    if (digitalRead(LIMIT_OPEN) == LOW) {
      if (iDebuglevel == 1 ) {
        Serial.printf(";Debug Line %i: digitalRead(LIMIT_OPEN) == LOW \n",__LINE__);  
        Serial.printf(";Debug Line %i: Already open\n",__LINE__);
        Serial.printf(";Debug Line %i: S:%i A:%i\n",__LINE__,iSpeed,iAcceleration);
      }
      Serial.printf("n:open\n");
      Serial.println("ok\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // Check for any passed parms and if get a rc=1 we had an invalid parm so exit.
    if (checkParms() == 1) {
      if (iDebuglevel == 1 ) {
        Serial.printf(";Debug Line %i: checkParms()\n",__LINE__);
      }
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      if (iDebuglevel == 1 ) {
        Serial.printf(";Debug Line %i: digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH\n",__LINE__);
      }
      Serial.printf(";Line %i: Magazine Slide Caught between Open/Close: trying to recover\n",__LINE__);
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {
        #ifdef __ENABLE_INA219__
        if (iDebuglevel == 1 ) {
          read_ina219();
        }
        #endif
      }
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.printf("error\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // No that the checks are done we can actually move the motor
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      if (iDebuglevel == 1 ) // if debuging print some information
      {
        Serial.printf(";Debug Line %i: digitalRead(LIMIT_CLOSE) == LOW\n",__LINE__);
        Serial.printf(";Debug Line %i: Not Open, Moving to open position\n",__LINE__);
        Serial.printf(";Debug Line %i: S:%i A: %i\n",__LINE__,iSpeed,iAcceleration);
      }

      time1 = millis();
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);

      if (iDebuglevel == 1 ) {
        Serial.printf(";Debug Line %i: S:%i A:%i D:%s\n",__LINE__,deltaprintr_motor.currentSpeed(),iAcceleration,deltaprintr_motor.currentDirection()?"backwards":"forward");
      }
      // Wait loop for limit to be hit
      while (digitalRead(LIMIT_OPEN) == HIGH) // This is the main loop that waits for the LIMIT_OPEN switch to close 
      {
        #ifdef __ENABLE_INA219__
          if ( iDebuglevel == 2 ) {
            read_ina219();
          }
        #endif
        time2 = millis(); // grap the current mS
        if (GCode.availableValue('M')) // check if a M112 is sent and if so ESTOP  
        {
          int code = (int) GCode.GetValue('M');
          if (code == 112) { // GOT ESTOP COMMAND
            Serial.printf(";Line %i: Received M112 to E-STOP\n",__LINE__);
            deltaprintr_motor.breakdown(0, 0);
            Serial.printf("error\n");
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
            Serial.printf(";Line %i: Emergincy Stop Triggered\n",__LINE__);
            Serial.printf("!estop\n");
            bEstop = true;
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
            return;
          }
          #endif
          if ((time2 - time1) >= ulTimeout) // check if we've timed out and if so set error and return
          {
            Serial.printf(";Line %i: Timed out opening Nozzle\n",__LINE__);
            Serial.printf("error\n");
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

        if (iDebuglevel == 3 ) {
          Serial.printf(";Debug Line %i: Move took %u(mS)\n",__LINE__,time2 - time1);
        }
      }
      // now let user know we're open and things are OK.
      #ifdef __ENABLE_INA219__
        fPeekCurrentmA = 0;
      #endif
      Serial.printf("n:open\n");
      Serial.println("ok");
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
        Serial.printf("E-STOP\n");
        Serial.printf("error\n");
        return;
      } 
      #endif 

      digitalWrite(ACTIVITYLED, HIGH);
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      Serial.println("M805");

      // If the LIMIT_CLOSE is already set just exit since we already are closed
      if (digitalRead(LIMIT_CLOSE) == LOW) {
        if ( iDebuglevel == 1 ) {      
          Serial.printf(";Debug Line %i: digitalRead(LIMIT_CLOSE) == LOW\n",__LINE__);
          Serial.printf(";Debug Line %i: Already closed\n",__LINE__);
          Serial.printf(";Debug Line %i: S:%i A:%i\n",__LINE__,iSpeed,iAcceleration);
        }
        Serial.printf("n:close\n");
        Serial.printf("ok\n");
        #ifdef __ENABLE_NEOPIXEL__
          neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        #endif
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // Check for any passed parms
      if (int i = checkParms() == 1) {
        if (iDebuglevel == 1 ) {
          Serial.printf(";Debug Line %i: checkParms() state=%i\n",__LINE__,i);
        }
        #ifdef __ENABLE_NEOPIXEL__
          neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        #endif
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // If Magazine is caught between Open/CLose force it to close
      if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
        Serial.printf(";Line %i: Magazine Slide Caught between Open/Close: trying to recover\n",__LINE__);
        deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);
        while (digitalRead(LIMIT_OPEN) == HIGH) {} // wait for LIMIT_OPEN switch to go low and soon as it does we 
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
        Serial.printf("error\n");
        bError = true;
        #ifdef __ENABLE_NEOPIXEL__
          neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
        #endif
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // Now that the checks are done we can actually move the motor
      if (digitalRead(LIMIT_OPEN) == LOW) {
        if (iDebuglevel == 1 ) {
          Serial.printf(";Debug Line %i: digitalRead(LIMIT_OPEN) == LOW\n",__LINE__);
          Serial.printf(";Debug Line %i: Not Closed, Moving to Closed Position\n",__LINE__);
          Serial.printf(";Debug Line %i: S:%i A:%i\n",__LINE__,iSpeed,iAcceleration);
        }
        time1 = millis();
        deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
        if (iDebuglevel == 1 ) {
          Serial.printf(";Debug Line %i: S:%i A:%i D:%s\n",__LINE__,deltaprintr_motor.currentSpeed(),iAcceleration,deltaprintr_motor.currentDirection()?"backwards":"forward");
        }
        // Wait loop for limit to be hit
        while (digitalRead(LIMIT_CLOSE) == HIGH) {
          #ifdef __ENABLE_INA219__
          if ( iDebuglevel == 2 ) {
          read_ina219();
          }
          time2 = millis();
          #endif
          // timetmp = time2-time1;
          #ifdef __ENABLE_ESTOP_SWITCH__
          if (Estopswitch.pressed()) {
            deltaprintr_motor.breakdown(0, 0);
            Serial.printf(";Line %i: Emergincy Stop Triggered\n",__LINE__);
            Serial.printf("error\n");
            bEstop = true;
            #ifdef __ENABLE_NEOPIXEL__
              neopixel_led(NEOPIXEL_RED, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
            #endif
          }
          #endif
          if ((time2 - time1) >= ulTimeout) {
            Serial.printf(";Line %i: Timed out opening Nozzle\n",__LINE__);
            Serial.printf("error\n");
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
      if (iDebuglevel == 3 ) {
        Serial.printf(";Debug Line %i: Move took %u(mS)\n",__LINE__,(time2-time1));
      }

      // now let user know we're open and things are OK.
      #ifdef __ENABLE_INA219__
        fPeekCurrentmA = 0;
      #endif 
      Serial.printf("n:closed\n");
      Serial.printf("ok\n");
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

    Serial.printf(";Open Button Pressed\n"); // echo the command
    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine

    if (bEstop == true) {
      Serial.printf("E-STOP\n");
      Serial.printf("error\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If the LIMIT_OPEN is already set just exit since we already are open
    if (digitalRead(LIMIT_OPEN) == LOW) {
      Serial.printf(";Line %i: Already Open\n",__LINE__);
      Serial.printf("ok\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      Serial.printf(";Line %i: Magazine Slide Caught between Open/Close: trying to recover\n",__LINE__);
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_BACKWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.printf(";error\n");
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
        if ( iDebuglevel == 1 ) {
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
      Serial.printf("n:open\n");
      Serial.printf("ok\n");
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

    Serial.printf(";Close Button Pressed\n"); // echo the command
    #ifdef __ENABLE_NEOPIXEL__
      neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_ON); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
    #endif
    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine

    if (bEstop == true) {
      Serial.printf("E-STOP\n");
      Serial.printf("ok\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
        digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If the LIMIT_CLOSE is already closed just exit since we already are closed
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      Serial.printf(";Line %i: Already Closed\n",__LINE__);
      Serial.printf("ok\n");
      #ifdef __ENABLE_NEOPIXEL__
        neopixel_led(NEOPIXEL_GREEN, NEOPIXEL_OFF); // LED IS BLUE DURING MAIN LOOP,  GREEN WHEN EXECUTING COMMAND AND RED ON ERROR
      #endif
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      Serial.printf(";Line %i: Magazine Slide Caught between Open/Close: trying to recover\n",__LINE__);
      deltaprintr_motor.drive(iSpeed, deltaprintr_motor.DIRECTION_FORWARD, iAcceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.printf(";error\n");
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
        if ( iDebuglevel == 1 ) {
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
      Serial.printf("n:closed\n");
      Serial.printf("ok\n");
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
  
  Serial.printf("Nozzle Magazine Controller (c) 2022 KBAstronomics\n");

  // Setup our INPUT/OUTPUTS
  pinMode(ACTIVITYLED, OUTPUT);
  pinMode(LIMIT_CLOSE, INPUT_PULLUP);
  pinMode(LIMIT_OPEN, INPUT_PULLUP);

  #ifdef __ENABLE_OC_SWITCH__
    // CREATE BUTTONS
    Openswitch.attach( OPEN_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    Closeswitch.attach( CLOSE_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    
    // DEBOUNCE INTERVAL IN MILLISECONDS
    Openswitch.interval(10);
    Closeswitch.interval(10); 
  
    // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
    Openswitch.setPressedState(LOW);
    Closeswitch.setPressedState(LOW); 
  #endif 
 
  // If the ESTOP is defined create the button with a 5ms debounce and active low 
  #ifdef __ENABLE_ESTOP_SWITCH__
    Estopswitch.attach( ESTOP_SWITCH ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    Estopswitch.interval(10);
    Estopswitch.setPressedState(LOW);
    bEstop = false;
  #endif

  // If the INA219 is defined set it up
  #ifdef __ENABLE_INA219__
    Wire.begin();
    if( !ina219.init() ) {
      Serial.printf(";Line %i: INA219 not connected!\n",__LINE__);
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
      Serial.printf("E-STOP Cleared\n");
      Serial.printf("ok\n");
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