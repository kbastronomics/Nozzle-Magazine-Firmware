/**********************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 0.2.0b
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/nzmag
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "nzholder.h"

//
// begin callback functions,   all implemented as blocking calls
//

//
// checkASB:   Checks for the A|B|S PARMS and stores the results
//
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
      if ( nSpeed < 0 || nSpeed > 5000 ) {
        Serial.println(";Error: S Value out of range"); 
        nSpeed=nSpeedtmp;
        Serial.println("!ERROR");
        return 1;
      }

      if ( nSpeed <= 30) { 
        nSpeed = 30; 
      }
    rc = 2;
  }
  // if [A<Milliseconds>] is sent get the value and save it
  if(GCode.availableValue('A'))   {
    nAcceleration = (int) GCode.GetValue('A');
      if (nAcceleration < 0 || nAcceleration > 1000 ) {
        Serial.println(";Error: A Value out of range");
        nAcceleration = nAccellerationtmp;
        Serial.println("!ERROR");
        return 1;
      }
   rc = 3;
  } 
  // if [B<value>] is sent get the value and save it
  if(GCode.availableValue('B'))   {
    nBreak = (int) GCode.GetValue('B');
      if (nBreak < 0 || nBreak > 100 ) {
        Serial.println(";Error: B Value out of range");
        nBreak = nBreaktmp;
        Serial.println("!ERROR");
        return 1;
      }
   rc = 4;
  } 
 Serial.println("!OK");
 return rc;
}

//
// G4_pause:  Delays for value S in milliseconds 
// Has no real effect due to blocking nature of calls
//
void G4_pause() {
  int delayvaule=0;

  Serial.println(">G4");
  // if [S<VALUE<] this is in seconds so multiple by 1000 
  if(GCode.availableValue('S'))   {
    delayvaule = (int) GCode.GetValue('S') * 1000;
  }

  // if [P<VALUE<] this is in milliseconds so use as-is
  if(GCode.availableValue('P'))   {
    delayvaule = (int) GCode.GetValue('P');
  }

  delay(delayvaule);
  Serial.println("!OK");
}

//
// M111_debug:  Sets Debug Flags P<MODULE> S<LEVEL>
//
void M111_debug() {

  digitalWrite(LEDpin, HIGH);
  Serial.println(">M111");

    // if [P<VALUE<] this is in milliseconds so use as-is
  if(GCode.availableValue('P'))   {
    debug_module = (int) GCode.GetValue('P');
  }

  // if [S<VALUE<] this is in seconds so multiple by 1000 
  if(GCode.availableValue('S'))   {
    debug_level = (int) GCode.GetValue('S');
  }

  Serial.print(";DEBUG P");
  Serial.print(debug_module);
  Serial.print(" S");
  Serial.println(debug_level);
  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
}

//
// M112_estop:  Cause an immediate stop to motion
//
void M112_estop() {

  digitalWrite(LEDpin, HIGH);
  Serial.println(">M112");
  deltaprintr_motor.brakedown(0,0);
  Serial.println("E-STOP");
  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
}

//
// M114_reportPostion:  Reports current postion information
//
void M114_reportPostion() {
  String state="";

  digitalWrite(LEDpin, HIGH);
  //  get current state of limit switch's
  Serial.println(">M114");
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    state = "Open"; 
  }
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
    state = "Closed"; 
  }

  if (digitalRead(LIMIT_OPEN) == HIGH && digitalRead(LIMIT_CLOSE) == HIGH ) {
    state = "JAMMED"; 
  }

  //Serial.print("N:");
  Serial.println(state);
  // Send final OK message
  if( state == "JAMMED") {
    Serial.println("!ERROR");
  } else {
    Serial.println("!OK");
  }

  digitalWrite(LEDpin, LOW);
}

//
// M115_reportFirmware:   Reports firmware information
//
void M115_reportFirmware() {
  char buffer[300];

  digitalWrite(LEDpin, HIGH);
  sprintf(buffer,"FIRMWARE_NAME:%s %s (Github) SOURCE_CODE_URL:https://github.com/kbastronomics/nzmag PROTOCOL_VERSION:%s MACHINE_TYPE:%s NOZZLE_COUNT:%d CONTROL_BOARD:%s MOTOR_DRIVE:%s UUID %s",FIRMWARE_NAME, FIRMWARE_VERSION, PROTOCOL_VERSION,MACHINE_TYPE,NOZZLE_COUNT,CONTROL_BOARD,MOTOR_DRIVE,UUID);
  Serial.println(">M115");
  Serial.println(buffer);
  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
}

//
// M119_endStopState: report the current state of the endstops
//
void M119_endStopState() {
  String n_min="OPEN", n_max="OPEN";

  digitalWrite(LEDpin, HIGH);
  Serial.println(">M119");
  
  if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH ) {
    Serial.println(";Magazine Slide Caught between Open/Close");
    Serial.println("!ERROR");
    return;
  }
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    n_max = "TRIGGERED"; 
  }
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
    n_min = "TRIGGERED"; 
  }

  Serial.println("Reporting Endstop State");
  Serial.print("n_min:");
  Serial.println(n_min);    // n_min is closed position
  Serial.print("n_max:");
  Serial.println(n_max);    // n_max is open position
  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
  return;
}


//
// M220_setFeedrate: Sets the Speed, Acceleration and Brake values 
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

  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
}

//
// M303_autotune: Perform autotune to find the best speed and acceleration values
//
void M303_autotune() {
  int count = 5;    // default number of cycles to find the optiman speed/acceleration values
  digitalWrite(LEDpin, HIGH);
  Serial.println(">M303");
  
    // if [C<value>] is sent get the value and save it
  if(GCode.availableValue('C'))   {
    count = (int) GCode.GetValue('C');
      if (count <= 0 || count > 1000 ) {
        Serial.println("Error: C Value out of range");
        count = 5;
        Serial.println("!OK");
        return;
      }
  } 

  // INSERT AUTOTUNING HERE
  for( int i=0; i <= count; i++ ) {
   GCode.comment('C',(double)count);
  }

  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
}

//
// M804_openNozzlemagazine:  Open the magazine or report open if already open
//
void M804_openNozzlemagazine() {
  unsigned long time1=0,time2=0,timetmp=0;

  digitalWrite(LEDpin, HIGH); // signal that we are in the routine
  Serial.println(">M804");     // echo the command

  // If Magazine is caught between Open/CLose force it to open
  if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH ) {
    if ( debug_module == 805 && debug_level == 1) {
      Serial.println("DEBUG: digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH"); 
    }

    Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing open");
    deltaprintr_motor.drive(nSpeed,deltaprintr_motor.DIRECTION_FORWARD, nAcceleration);
    while( digitalRead(LIMIT_OPEN) == HIGH )  { }
    // wait for LIMIT_OPEN switch to go low and soon as it does we 
    deltaprintr_motor.brakedown(0,0);               // Default is Hard Stop (should I allow this to be overridden???)
    Serial.println("!OK");
    digitalWrite(LEDpin, LOW);
    return;
  }

    // Check for any passed parms and if get a rc=1 we had an invalid parm so exit.
  if ( checkASB() == 1 ) {
    if ( debug_module == 805 && debug_level == 1) {
      Serial.println("DEBUG: checkASB()"); 
    }
    digitalWrite(LEDpin, LOW);
    return;
  }

  // If the LIMIT_OPEN is already set just so so and exit
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    if ( debug_module == 805 && debug_level == 1) {
      Serial.println("DEBUG: digitalRead(LIMIT_OPEN) == LOW "); 
      Serial.print("DEBUG: S:");
      Serial.print(nSpeed);
      Serial.print(" A:");
      Serial.println(nAcceleration);
    } 
    Serial.println("Open");
    Serial.println("!OK");
    digitalWrite(LEDpin, LOW);
    return;
  }

  // start by checking if we on the limit closed swtich (which we should be) and if so tell motor to move
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
  // execute gcode command
  time1 = millis();
  deltaprintr_motor.drive(nSpeed, deltaprintr_motor.DIRECTION_FORWARD, nAcceleration);
    if ( debug_module == 804 && debug_level == 1) {
      Serial.print("DEBUG: S:");
      Serial.print(deltaprintr_motor.currentSpeed());
      Serial.print(" A:");
      Serial.print(nAcceleration);
      Serial.print(" D:");
      Serial.println(deltaprintr_motor.currentDirection());
    }
    while( digitalRead(LIMIT_OPEN) == HIGH )  { 
      time2 = millis();
      timetmp = time2-time1;
      if( timetmp >= 1000 ) {
        Serial.println("Timed out opening Nozzle");
        Serial.println("!ERROR");
        break;
      }
    } // wait for LIMIT_OPEN switch to go low and soon as it does we 
    deltaprintr_motor.brakedown(0,0);             // Hard Stop the motor
  }
  time2 = millis();
  if ( debug_module == 804 && debug_level == 1) {
    Serial.print(";move took (mS): ");
    Serial.println(time2-time1);
  }
   // now let user know we're open and things are OK.
  Serial.println("Open");
  Serial.println("!OK");
  digitalWrite(LEDpin, LOW); // turn LED off to show we're out of the routine
}

//
// M805_closeNozzlemagazine: Close the magazine or report open if already closed
//
void M805_closeNozzlemagazine() {
  unsigned long time1=0,time2=0,timetmp=0;
 
  digitalWrite(LEDpin, HIGH);
  Serial.println(">M805");

  // If Magazine is caught between Open/CLose force it to close
  if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH ) {
    if ( debug_module == 805 && debug_level == 1) {
    Serial.println("DEBUG: digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH"); 
    }
    Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing close");
    deltaprintr_motor.drive(nSpeed,deltaprintr_motor.DIRECTION_BACKWARD, nAcceleration);
    while( digitalRead(LIMIT_CLOSE) == HIGH )  { }
    // wait for LIMIT_OPEN switch to go low and soon as it does we 
    deltaprintr_motor.brakedown(0,0);               // Default is Hard Stop (should I allow this to be overridden???)

    Serial.println("!OK");
    digitalWrite(LEDpin, LOW);
    return;
  }

  // If the LIMIT_CLOSE is already set just so so and exit
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
    if ( debug_module == 805 && debug_level == 1) {
    Serial.println("DEBUG: digitalRead(LIMIT_CLOSE) == LOW"); 
    }
    Serial.println("Closed");
    Serial.println("!OK");
    digitalWrite(LEDpin, LOW);
    return;
  }

  // Check for any passed parms
  if ( checkASB() == 1 ) {
    if ( debug_module == 805 && debug_level == 1) {
      Serial.println("DEBUG: checkASB()"); 
    }
    digitalWrite(LEDpin, LOW);
    return;
  }

  // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
  if (digitalRead(LIMIT_OPEN) == LOW ) {
    if ( debug_module == 805 && debug_level == 1) {
      Serial.println("DEBUG: digitalRead(LIMIT_OPEN) == LOW "); 
      Serial.print("DEBUG: S:");
      Serial.print(nSpeed);
      Serial.print(" A:");
      Serial.println(nAcceleration);
     } 
    time1 = millis();
    deltaprintr_motor.drive(nSpeed,deltaprintr_motor.DIRECTION_BACKWARD, nAcceleration);
      if ( debug_module == 805 && debug_level == 1) {
        Serial.print("DEBUG: S:");
        Serial.print(deltaprintr_motor.currentSpeed());
        Serial.print(" A:");
        Serial.print(nAcceleration);
        Serial.print(" D:");
        Serial.println(deltaprintr_motor.currentDirection());
      }
      while( digitalRead(LIMIT_CLOSE) == HIGH )  { 
        time2 = millis();
        timetmp = time2-time1;
        if( timetmp >= TIMEOUT ) {
          Serial.println("Timed out opening Nozzle");
          Serial.println("!ERROR");
          break;
        }
      } // wait for LIMIT_OPEN switch to go low and soon as it does we 
    deltaprintr_motor.brakedown(0,0);               // Default is Hard Stop (should I allow this to be overridden???)
  }   

  time2 = millis();
  if ( debug_module == 805 && debug_level == 1) {
    Serial.print("DEBUG: Move took (mS): ");
    Serial.println(time2-time1);
  }
  
  // now let user know we're open and things are OK.
  Serial.println("Closed");
  Serial.println("!OK");
  digitalWrite(LEDpin, LOW);
}

void setup() { 
  GCode.begin(115200, ">"); //responce => ok, rs or !!
  pinMode(LEDpin, OUTPUT);
  pinMode(LIMIT_CLOSE,INPUT_PULLUP);
  pinMode(LIMIT_OPEN,INPUT_PULLUP);
  pinMode(ESTOP,INPUT_PULLUP);
}

void loop() {

  GCode.available();

  // if( digitalRead(ESTOP) == LOW) {
  //   deltaprintr_motor.brakedown(0,0);
  //   Serial.println("E-STOP");
  //   Serial.println("!ERROR");
  // }

}