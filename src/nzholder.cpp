/**********************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 0.0.6b
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/nzmag
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "nzholder.h"

// begin callback functions,   all implemented as blocking calls
int checkASB() {
  int nSpeedtmp=nSpeed;
  int nAccellerationtmp=nAcceleration;
  int nBreaktmp=nBreak;
  int rc=0;

// rc=0 no parms passed
// rc=1 error in parms
// rc=2 parm passed

// if [S<percent>] is sent get the value and save it
  if(GCode.availableValue('S'))   {
    nSpeed = (int) GCode.GetValue('S');
      if ( nSpeed < 0 || nSpeed > 100 ) {
        GCode.comment("Error: S Value out of range"); 
        nSpeed=nSpeedtmp;
        GCode.comment("OK");
        return 1;
      }
    rc = 2;
  }
  // if [A<percent>] is sent get the value and save it
  if(GCode.availableValue('A'))   {
    nAcceleration = (int) GCode.GetValue('A');
      if (nAcceleration < 0 || nAcceleration > 100 ) {
        GCode.comment("Error: A Value out of range");
        nAcceleration = nAccellerationtmp;
        GCode.comment("OK");
        return 1;
      }
   rc = 3;
  } 
  // if [B<value>] is sent get the value and save it
  if(GCode.availableValue('B'))   {
    nBreak = (int) GCode.GetValue('B');
      if (nBreak < 0 || nBreak > 100 ) {
        GCode.comment("Error: B Value out of range");
        nBreak = nBreaktmp;
        GCode.comment("OK");
        return 1;
      }
   rc = 4;
  } 
 return rc;
}


void M112_estop() {

  digitalWrite(LEDpin, HIGH);
  GCode.comment(">M112");
  deltaprintr_motor.breakdown(nBreak);
  GCode.comment("E-STOP");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}

void M114_reportPostion() {
  String state="";

  digitalWrite(LEDpin, HIGH);
  //  get current state of limit switch's
  GCode.comment(">M114");
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    state = "Open"; 
  }
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
    state = "Closed"; 
  }
  // Send report
  Serial.print("N:");
  Serial.println(state);
  // Send final OK message
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}


void M115_reportFirmware() {
  char buffer[300];

  digitalWrite(LEDpin, HIGH);
  sprintf(buffer,"FIRMWARE_NAME:%s %s (Github) SOURCE_CODE_URL:https://github.com/kbastronomics/nzmag PROTOCOL_VERSION:%s MACHINE_TYPE:%s NOZZLE_COUNT:%d CONTROL_BOARD:%s MOTOR_DRIVE:%s",FIRMWARE_NAME, FIRMWARE_VERSION, PROTOCOL_VERSION,MACHINE_TYPE,NOZZLE_COUNT,CONTROL_BOARD,MOTOR_DRIVE);
  GCode.comment(">M115");
  Serial.println(buffer);
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}



void M119_endStopState() {
  String n_min="OPEN", n_max="OPEN";

  digitalWrite(LEDpin, HIGH);
  GCode.comment(">M119");
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    n_max = "TRIGGERED"; 
  }

  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
    n_min = "TRIGGERED"; 
  }

  GCode.comment("Reporting Endstop State");
  Serial.print("n_min:");
  Serial.println(n_min);    // n_min is closed position
  Serial.print("n_max:");
  Serial.println(n_max);    // n_max is open position
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}

//
// M220_setFeedrate
// FUNCTION SETS GLOBAL VARIABLES ONLY
//
void M220_setFeedrate() {

  digitalWrite(LEDpin, HIGH);

  if ( checkASB() == 1) { // ERROR So do nothing
    return;
  }

  // echo gcode sent
  Serial.println(">M220");
  Serial.print("S:");
  Serial.print(nSpeed);
  Serial.print(" A:");
  Serial.print(nAcceleration);
  Serial.print(" B:");
  Serial.println(nBreak);

  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}

void M804_openNozzleholdder() {

  digitalWrite(LEDpin, HIGH);
  GCode.comment(">M804");

   if ( checkASB() == 1 ) {
    return;
  }
  
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
  // execute gcode command
  deltaprintr_motor.drive(nSpeed, deltaprintr_motor.DIRECTION_FORWARD, nAcceleration);
  if ( DEBUG == 1 ) {
    Serial.print("S:");
    Serial.print(deltaprintr_motor.currentSpeed());
    Serial.print(" A:");
    Serial.print(nAcceleration);
    Serial.print(" D:");
    Serial.println(deltaprintr_motor.currentDirection());
  }
  while( digitalRead(LIMIT_OPEN) == HIGH )  { } // wait for LIMIT_OPEN switch to hit and ignore the LIMIT_CLOSE switch 
  deltaprintr_motor.breakdown(nBreak);   // Hard Stop (should I allow this to be overridden???)
  }
  GCode.comment("Open");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}

void M805_closeNozzleholdder() {
 
  digitalWrite(LEDpin, HIGH);
  GCode.comment(">M805");
  
  if ( checkASB() == 1 ) {
    return;
  }
  // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    deltaprintr_motor.drive(nSpeed,deltaprintr_motor.DIRECTION_BACKWARD, nAcceleration);
    if ( DEBUG == 1 ) {
      Serial.print("S:");
      Serial.print(deltaprintr_motor.currentSpeed());
      Serial.print(" A:");
      Serial.print(nAcceleration);
      Serial.print(" D:");
      Serial.println(deltaprintr_motor.currentDirection());
    }
    while( digitalRead(LIMIT_CLOSE) == HIGH ) { }
    deltaprintr_motor.breakdown(nBreak);  // Default is Hard Stop (should I allow this to be overridden???)
  }   
  GCode.comment("Closed");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
}

// begin setup

void setup() { 
  GCode.begin(115200, ">"); //responce => ok, rs or !!
  pinMode(LEDpin, OUTPUT);
  pinMode(LIMIT_CLOSE,INPUT_PULLUP);
  pinMode(LIMIT_OPEN,INPUT_PULLUP);
}

// begin loop
void loop() {

  GCode.available();

}