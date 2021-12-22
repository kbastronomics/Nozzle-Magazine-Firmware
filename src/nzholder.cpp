/**********************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 1.0
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/smgvbest/nzholder
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "nzholder.h"

// begin callback functions,   all implemented as blocking calls
void estop() {
  // M112
  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();
  GCode.comment(">M112");
  deltaprintr_motor.breakdown(nBreak);
  GCode.comment("E-STOP");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
  // strip.SetPixelColor(1, black);
  // strip.Show();
}

void reportFirmware() {
  // M115 report firmware 
  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();
  GCode.comment(">M115");
  Serial.println("FIRMWARE_NAME:Nozzle Magazine 1.0.0 (Github) SOURCE_CODE_URL:https://github.com/smgvbest/nzmag PROTOCOL_VERSION:1.0 MACHINE_TYPE:deltaprintr nozzle magazine NOZZLE_COUNT:20");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
  // strip.SetPixelColor(1, black);
  // strip.Show();
}


void reportPostion() {
  String state="";
  // M114 
  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();
  //  get current state of limit switch's
  GCode.comment(">M114");
  if (digitalRead(LIMIT_OPEN) == LOW ) { 
    state = "Open"; 
  }
  if (digitalRead(LIMIT_CLOSE) == LOW ) { 
    state = "Closed"; 
  }
  // Send report
  GCode.comment("Reporting Current Position");
  Serial.print("N:");
  Serial.println(state);
  // Send final OK message
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
  // strip.SetPixelColor(1, black);
  // strip.Show();
}

void endStopState() {
  // M119 
  String n_min="OPEN", n_max="OPEN";

  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();
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
  // strip.SetPixelColor(1, black);
  // strip.Show();
}

int checkAS() {
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
        // strip.SetPixelColor(1, red);
        // strip.Show();
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
        // strip.SetPixelColor(1, red);
        // strip.Show();
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
        // strip.SetPixelColor(1, red);
        // strip.Show();
        return 1;
      }
   rc = 4;
  } 

 return rc;
}

void setFeedrate() {
  // M220 [S<percent>] [A<percent>]

  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();

  if ( checkAS() == 1) { // ERROR So do nothing
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
  // strip.SetPixelColor(1, black);
  // strip.Show();
}

void openNozzleholdder() {
  // M804  [S<percent>] [A<percent>]

  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();
  GCode.comment(">M804");

   if ( checkAS() == 1 ) {
    return;
  }
  
  // execute gcode command
  deltaprintr_motor.drive(nSpeed, deltaprintr_motor.DIRECTION_FORWARD, nAcceleration);

  // report command
  Serial.print("S:");
  Serial.print(deltaprintr_motor.currentSpeed());
  Serial.print(" A:");
  Serial.print(nAcceleration);
  Serial.print(" D:");
  Serial.println(deltaprintr_motor.currentDirection());

  while( digitalRead(LIMIT_OPEN) == HIGH )  { } // wait for LIMIT_OPEN switch to hit and ignore the LIMIT_CLOSE switch 
  deltaprintr_motor.breakdown(nBreak);   // Hard Stop (should I allow this to be overridden???)
  GCode.comment("Open");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
  // strip.SetPixelColor(1, black);
  // strip.Show();
}

void closeNozzleholdder() {
  // M805 [S<percent>] [A<percent>]
 
  digitalWrite(LEDpin, HIGH);
  // strip.SetPixelColor(1, green);
  // strip.Show();
  GCode.comment(">M805");
  
  if ( checkAS() == 1 ) {
    return;
  }
  
  deltaprintr_motor.drive(nSpeed,deltaprintr_motor.DIRECTION_BACKWARD, nAcceleration);
  Serial.print("S:");
  Serial.println(deltaprintr_motor.currentSpeed());
  Serial.print(" D:");
  Serial.println(deltaprintr_motor.currentDirection());
  while( digitalRead(LIMIT_CLOSE) == HIGH ) { }
  deltaprintr_motor.breakdown(nBreak);  // Default is Hard Stop (should I allow this to be overridden???)
  GCode.comment("Closed");
  GCode.comment("OK");
  digitalWrite(LEDpin, LOW);
  // strip.SetPixelColor(1, black);
  // strip.Show();
}

// begin setup

void setup() { 
  // put your setup code here, to run once:
  // Serial.begin(19200);  
  // Serial.println("Nozzle Magazine V1 G-Code Driver");
  GCode.begin(19200, ">"); //responce => ok, rs or !!
  pinMode(LEDpin, OUTPUT);
  pinMode(LIMIT_CLOSE,INPUT_PULLUP);
  pinMode(LIMIT_OPEN,INPUT_PULLUP);

  // strip.Begin();
  // strip.ClearTo(green);
  // strip.Show();
}

// begin loop
void loop() {
  // put your main code here, to run repeatedly:

  GCode.available();

}