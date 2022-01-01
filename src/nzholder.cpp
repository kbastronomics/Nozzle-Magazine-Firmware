/**************************************************************************************************
 * Deltaprinter Nozzle Magazine Driver - version 0.8.0B
 * by Sandra Carroll <smgvbest@gmail.com> https://github.com/kbastronomics/Nozzle-Magazine-Firmware
 *
 * This firmware is licensed under a GPLv3 License
 **************************************************************************************************/

// Uncomment to use these features
#define __ENABLE_ESTOP_SWITCH__    // Add a ESTOP switch 
//#define __ENABLE_INA219__          // add current monitoring
//#define __ENABLE_EEPROM__          // Add EEPROM to store settings

#include "nzholder.h"

//
// BEGIN Support Functions
//

#ifdef __ENABLE_INA219__
//
// read_ina219:
//
void read_ina219() {
  // Read voltage and current from INA219.

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  // record the peek current used will be reset after a M114 is issued
  if (currrent_mA > peekCurrent_mA)
    peekCurrent_mA = current_mA;

  // Compute load voltage, power, and milliamp-hours.
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  power_mW = loadvoltage * current_mA;
  total_mA += current_mA;
  total_sec += 1;
  total_mAH = total_mA / 3600.0;
}
#endif

//
// checkParms:   Checks for the A|B|S PARMS and stores the results
//
int checkParms() {
  int _nSpeedtmp = _speed;
  int _nAccellerationtmp = _acceleration;
  int _nBreaktmp = _brake;
  int _timeouttmp = _timeout;
  int rc = 0;

  // rc=0 no parms passed
  // rc=1 error in any of the parms parms
  // rc=2 parm passed

  // if [S<percent>] is sent get the value and save it
  if (GCode.availableValue('S')) {
    _speed = (int) GCode.GetValue('S');
    if (_speed < 30 || _speed >= 255) {
      Serial.println(";Error: S Value out of range");
      _speed = _nSpeedtmp;
      Serial.println("!ERROR");
      return 1;
    }
    rc = 2;
  }
  // if [A<Milliseconds>] is sent get the value and save it
  if (GCode.availableValue('A')) {
    _acceleration = (int) GCode.GetValue('A');
    if (_acceleration < 0 || _acceleration > 1000) {
      Serial.println(";Error: A Value out of range");
      _acceleration = _nAccellerationtmp;
      Serial.println("!ERROR");
      return 1;
    }
    rc = 2;
  }
  // if [B<value>] is sent get the value and save it
  if (GCode.availableValue('B')) {
    _brake = (int) GCode.GetValue('B');
    if (_brake < 0 || _brake > 100) {
      Serial.println(";Error: B Value out of range");
      _brake = _nBreaktmp;
      Serial.println("!ERROR");
      return 1;
    }
    rc = 2;
  }

  // if [T<value>] is sent get the value and save it
  if (GCode.availableValue('T')) {
    _timeout = (int) GCode.GetValue('T');
    if (_timeout < 0 || _timeout > 10000) {
      Serial.println(";Error: T Value out of range");
      _timeout = _timeouttmp;
      Serial.println("!ERROR");
      return 1;
    }
    rc = 2;
  }

  // Serial.println("!OK");
  return rc;
}
//
// END Support Functions
//

//
// BEGIN callback functions,   all implemented as blocking calls
//

//
// G4_pause:  Delays for value S in milliseconds 
// Has no real effect due to blocking nature of calls
//
void G4_pause() {
  int delayvaule = 0;

  Serial.println(">G4");
  // if [S<VALUE<] this is in seconds so multiple by 1000 
  if (GCode.availableValue('S')) {
    delayvaule = (int) GCode.GetValue('S') * 1000;
  }

  // if [P<VALUE<] this is in milliseconds so use as-is
  if (GCode.availableValue('P')) {
    delayvaule = (int) GCode.GetValue('P');
  }

  delay(delayvaule); // Now wait for that time period
  Serial.println("!OK");
}

//
// T1_test:  Various timming tests
//
void T1_test() {
  unsigned long t1 = 0, t2 = 0, t3 = 0;
  int read = 0;

  Serial.println(">T1");

  t1 = micros(); // get start micros
  // function to test goes here
  read = digitalRead(LIMIT_CLOSE);
  t2 = micros(); // get end micros
  t3 = (t2 - t1) - 2; // micros() takes about 2uS to execute so subtract it

  // print results
  Serial.print(";digitalRead() time (t3=t2-t1): ");
  Serial.print(t3);
  Serial.println(" uS");
  Serial.println("!OK");
}

//
// M111_debug:  Sets Debug Flags P<MODULE> S<LEVEL>
//
void M111_debug() {

  digitalWrite(ACTIVITYLED, HIGH);
  Serial.println(">M111");

  // if [P<VALUE<] this is in milliseconds so use as-is
  if (GCode.availableValue('P')) {
    _debug_module = (int) GCode.GetValue('P');
  }

  // if [L<VALUE<] this is debug level (L0=OFF, L1=MODULE ONLY, L2=ALL) 
  if (GCode.availableValue('L')) {
    _debug_level = (int) GCode.GetValue('L');
  }

  Serial.print(";DEBUG P");
  Serial.print(_debug_module);
  Serial.print(" L");
  Serial.println(_debug_level);
  Serial.println("!OK");
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M112_estop:  Cause an immediate stop to motion
//
void M112_estop() {

  digitalWrite(ACTIVITYLED, HIGH);
  Serial.println(">M112");
  deltaprintr_motor.breakdown(0, 0);
  Serial.println("E-STOP");
  Serial.println("!OK");
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M114_reportPostion:  Reports current postion information
//
void M114_reportPostion() {
  String state = "";
  int istate = 0;

  digitalWrite(ACTIVITYLED, HIGH);
  //  get current state of limit switch's
  Serial.println(">M114");
  if (digitalRead(LIMIT_OPEN) == LOW) {
    state = "Open";
  }
  if (digitalRead(LIMIT_CLOSE) == LOW) {
    state = "Closed";
  }

  // if both limits are open we're stuck between them, allrt user
  if (digitalRead(LIMIT_OPEN) == HIGH && digitalRead(LIMIT_CLOSE) == HIGH) {
    state = "Magazine Slide Stuck between Open/Close";
    istate = 1;
  }

  // if both limits are closed we have a issue, alert user
  if (digitalRead(LIMIT_OPEN) == LOW && digitalRead(LIMIT_CLOSE) == LOW) {
    state = "Magazine Slide Limit Switch Shorted";
    istate = 2;

  }

  //Serial.print("N:");
  Serial.println(state);
  // Send final OK message
  if (istate > 0) {
    Serial.print("E:");
    Serial.println(istate);
    Serial.println("!ERROR");
  } else {
    Serial.println("!OK");
  }

  // if we have the ISA219 in use we'll report current stats
  #ifdef __USE_INA219__
  read_ina219();
  Serial.print("shuntvoltage:");
  Serial.println(shuntvoltage);
  Serial.print("busvoltage:");
  Serial.println(busvoltage);
  Serial.print("current_mA:");
  Serial.println(current_mA);
  Serial.print("peekCurrent_mA:");
  Serial.println(peekCurrent_mA);
  Serial.print("loadvoltage:");
  Serial.println(loadvoltage);
  Serial.print("power_mW:");
  Serial.println(power_mW);
  Serial.print("total_mA:");
  Serial.println(total_mA);
  Serial.print("total_sec:");
  Serial.println(total_sec);
  Serial.print("total_mAH:");
  Serial.println(total_mAH);
  peekCurrent_mA = 0;
  #endif

  digitalWrite(ACTIVITYLED, LOW);
}

//
// M115_reportFirmware:   Reports firmware information
//
void M115_reportFirmware() {
  char buffer[512];

  digitalWrite(ACTIVITYLED, HIGH);
  sprintf(buffer, "FIRMWARE_NAME: %s FIRMWARE_VERSION: %s (Github) SOURCE_CODE_URL: %s MACHINE_TYPE: %s NOZZLE_COUNT: %d CONTROL_BOARD: %s MOTOR_DRIVE: %s CURRENT_MONITOR: %s UUID: %s", FIRMWARE_NAME, FIRMWARE_VERSION, GITHUB_URL, MACHINE_TYPE, NOZZLE_COUNT, CONTROL_BOARD, MOTOR_DRIVER, CURRENT_MONITOR, UUID);
  Serial.println(">M115");
  Serial.println(buffer);
  Serial.println("!OK");
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M119_endStopState: report the current state of the endstops
//
void M119_endStopState() {
  String n_min = "OPEN", n_max = "OPEN";

  digitalWrite(ACTIVITYLED, HIGH);
  Serial.println(">M119");

  if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
    Serial.println(";Magazine Slide Caught between Open/Close");
    Serial.println("!ERROR");
    return;
  }
  if (digitalRead(LIMIT_OPEN) == LOW) {
    n_max = "TRIGGERED";
  }
  if (digitalRead(LIMIT_CLOSE) == LOW) {
    n_min = "TRIGGERED";
  }

  Serial.println("Reporting Endstop State");
  Serial.print("n_min:");
  Serial.println(n_min); // n_min is closed position
  Serial.print("n_max:");
  Serial.println(n_max); // n_max is open position
  Serial.println("!OK");
  digitalWrite(ACTIVITYLED, LOW);
  return;
}

//
// M220_setFeedrate: Sets the Speed, Acceleration and Brake values 
//
void M220_setFeedrate() {

  digitalWrite(ACTIVITYLED, HIGH);

  if (checkParms() == 1) { // ERROR So do nothing
    return;
  }

  // echo gcode sent
  Serial.println(">M220");
  Serial.print("S:");
  Serial.print(_speed);
  Serial.print(" A:");
  Serial.print(_acceleration);
  Serial.print(" B:");
  Serial.print(_brake);
  Serial.print(" T:");
  Serial.println(_timeout);

  Serial.println("!OK");
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M303_autotune: Perform autotune to find the best speed and acceleration values
//
void M303_autotune() {
  int count = 5; // default number of cycles to find the optiman speed/acceleration values
  digitalWrite(ACTIVITYLED, HIGH);
  Serial.println(">M303");

  // if [C<value>] is sent get the value and save it
  if (GCode.availableValue('C')) {
    count = (int) GCode.GetValue('C');
    if (count <= 0 || count > 1000) {
      Serial.println("Error: C Value out of range");
      count = 5;
      Serial.println("!OK");
      return;
    }
  }

  // INSERT AUTOTUNING HERE
  for (int i = 0; i <= count; i++) {
    #ifdef __ENABLE_ESTOP_SWITCH__
    if (estop_button.pressed()) {
      deltaprintr_motor.breakdown(0, 0);
      Serial.println(";Emergincy Stop Triggered");
      Serial.println("!OK");
    }
    #endif
    GCode.comment('C', (double) count);
  }

  Serial.println("!OK");
  digitalWrite(ACTIVITYLED, LOW);
}

//
// M804_openNozzlemagazine:  Open the magazine or report open if already open
//
void M804_openNozzleMagazine() {
    unsigned long time1 = 0, time2 = 0;

    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine
    Serial.println(">M804"); // echo the command

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      if ((_debug_module == 804 && _debug_level == 1) || _debug_level == 2) {
        Serial.println("DEBUG:490 digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH");
      }
      Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
      deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_BACKWARD, _acceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.println("!ERROR");
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If the LIMIT_OPEN is already set just exit since we already are open
    if (digitalRead(LIMIT_OPEN) == LOW) {
      if ((_debug_module == 804 && _debug_level == 1) || _debug_level == 2) {
        Serial.println("DEBUG:424 digitalRead(LIMIT_OPEN) == LOW ");
        Serial.print("DEBUG:425 S:");
        Serial.print(_speed);
        Serial.print(" A:");
        Serial.println(_acceleration);
      }
      Serial.println("Open");
      Serial.println("!OK");
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // Check for any passed parms and if get a rc=1 we had an invalid parm so exit.
    if (checkParms() == 1) {
      if ((_debug_module == 804 && _debug_level == 1) || _debug_level == 2) {
        Serial.println("DEBUG:439 checkParms()");
      }
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      if ((_debug_module == 804 && _debug_level == 1) || _debug_level == 2) // if debuging print some information
      {
        Serial.println("DEBUG:456 digitalRead(LIMIT_CLOSE) == LOW ");
        Serial.print("DEBUG:457 S:");
        Serial.print(_speed);
        Serial.print(" A:");
        Serial.println(_acceleration);
      }

      time1 = millis();
      deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_FORWARD, _acceleration);

      if ((_debug_module == 804 && _debug_level == 1) || _debug_level == 2) {
        Serial.print("DEBUG:468 S:");
        Serial.print(deltaprintr_motor.currentSpeed());
        Serial.print(" A:");
        Serial.print(_acceleration);
        Serial.print(" D:");
        Serial.println(deltaprintr_motor.currentDirection());
      }
      while (digitalRead(LIMIT_OPEN) == HIGH) // This is the main loop that waits for the LIMIT_OPEN switch to close 
      {
        time2 = millis(); // grap the current mS
        if (GCode.availableValue('M')) // check if a M112 is sent and if so ESTOP  
        {
          int code = (int) GCode.GetValue('M');
          if (code == 112) { // GOT ESTOP COMMAND
            Serial.println(";Received M112 to E-STOP");
            deltaprintr_motor.breakdown(0, 0);
            Serial.println("!OK");
            digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
            return; // Since we E-STOP we just exit              }
          }
        }
          #ifdef __ENABLE_ESTOP_SWITCH__ // if we're using the ESTOP SWITCH check the ping and stop then return
          if (estop_button.pressed()) {
            deltaprintr_motor.breakdown(0, 0);
            Serial.println(";Emergincy Stop Triggered");
            Serial.println("!OK");
            return;
          }
          #endif
          if ((time2 - time1) >= _timeout) // check if we've timed out and if so set error and return
          {
            Serial.println(";Timed out opening Nozzle");
            Serial.println("!ERROR");
            deltaprintr_motor.breakdown(0, 0);
            digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
            return;
          }
        } // ends that main loop for the LIMIT_OPEN
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
        time2 = millis();

        if ((_debug_module == 804 && _debug_level == 1) || _debug_level == 2) {
          Serial.print("DEBUG:501 Move took (mS): ");
          Serial.println(time2 - time1);
        }

      }
      // now let user know we're open and things are OK.
      Serial.println("Open");
      Serial.println("!OK");
      digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
}

//
// M805_closeNozzlemagazine: Close the magazine or report open if already closed
//
 void M805_closeNozzleMagazine() {
      unsigned long time1 = 0, time2 = 0;

      digitalWrite(ACTIVITYLED, HIGH);
      Serial.println(">M805");

      // If the LIMIT_CLOSE is already set just exit since we already are closed
      if (digitalRead(LIMIT_CLOSE) == LOW) {
        if ((_debug_module == 805 && _debug_level == 1) || _debug_level == 2) {
          Serial.println("DEBUG:552 digitalRead(LIMIT_CLOSE) == LOW ");
          Serial.print("DEBUG:553 S:");
          Serial.print(_speed);
          Serial.print(" A:");
          Serial.println(_acceleration);
        }
        Serial.println("Open");
        Serial.println("!OK");
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // If Magazine is caught between Open/CLose force it to close
      if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
        if ((_debug_module == 805 && _debug_level == 1) || _debug_level == 2) {
          Serial.println("DEBUG:537 digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH");
        }
        Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
        deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_FORWARD, _acceleration);
        while (digitalRead(LIMIT_OPEN) == HIGH) {}
        // wait for LIMIT_OPEN switch to go low and soon as it does we 
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
        Serial.println("!OK");
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // Check for any passed parms
      if (checkParms() == 1) {
        if ((_debug_module == 805 && _debug_level == 1) || _debug_level == 2) {
          Serial.println("DEBUG:567 checkASB()");
        }
        digitalWrite(ACTIVITYLED, LOW);
        return;
      }

      // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
      if (digitalRead(LIMIT_OPEN) == LOW) {
        if ((_debug_module == 805 && _debug_level == 1) || _debug_level == 2) {
          Serial.println("DEBUG:576 digitalRead(LIMIT_OPEN) == LOW ");
          Serial.print("DEBUG:577 S:");
          Serial.print(_speed);
          Serial.print(" A:");
          Serial.println(_acceleration);
        }
        time1 = millis();
        deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_BACKWARD, _acceleration);
        if ((_debug_module == 805 && _debug_level == 1) || _debug_level == 2) {
          Serial.print("DEBUG:585 S:");
          Serial.print(deltaprintr_motor.currentSpeed());
          Serial.print(" A:");
          Serial.print(_acceleration);
          Serial.print(" D:");
          Serial.println(deltaprintr_motor.currentDirection());
        }
        while (digitalRead(LIMIT_CLOSE) == HIGH) {
          time2 = millis();
          // timetmp = time2-time1;
          #ifdef __ENABLE_ESTOP_SWITCH__
          if (estop_button.pressed()) {
            deltaprintr_motor.breakdown(0, 0);
            Serial.println(";Emergi1ncy Stop Triggered");
            Serial.println("!OK");
          }
          #endif
          if ((time2 - time1) >= _timeout) {
            Serial.println(";Timed out opening Nozzle");
            Serial.println("!ERROR");
            deltaprintr_motor.breakdown(0, 0);
            return;
          }
        } // wait for LIMIT_OPEN switch to go low and soon as it does we 
        deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      }
      time2 = millis();
      if ((_debug_module == 805 && _debug_level == 1) || _debug_level == 2) {
        Serial.print("DEBUG:613 Move took (mS): ");
        Serial.println(time2 - time1);
      }

      // now let user know we're open and things are OK.
      Serial.println("Closed");
      Serial.println("!OK");
      digitalWrite(ACTIVITYLED, LOW);
    }
//
// END callback functions,   all implemented as blocking calls
//


//
// openNozzle:  Manuualy open the Nozzle via push button on controller
//
void openNozzleMagazine() {

    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine
    Serial.println(";Open Button Pressed"); // echo the command

    // If the LIMIT_OPEN is already set just exit since we already are open
    if (digitalRead(LIMIT_OPEN) == LOW) {
      Serial.println(";Already Open");
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
      deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_BACKWARD, _acceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.println(";ERROR");
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_FORWARD, _acceleration);

      while (digitalRead(LIMIT_OPEN) == HIGH) { } // ends that main loop for the LIMIT_OPEN
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
    }
      // now let user know we're open and things are OK.
      Serial.println(";Nozzle Open");
      digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
}

//
// closeNozzle:  Manuualy close the Nozzle via push button on controller
//
void closeNozzleMagazine() {

    digitalWrite(ACTIVITYLED, HIGH); // signal that we are in the routine
    Serial.println(";Button Close Pressed"); // echo the command

    // If the LIMIT_CLOSE is already closed just exit since we already are closed
    if (digitalRead(LIMIT_CLOSE) == LOW) {
      Serial.println(";Already Closed");
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // If Magazine is caught between Open/CLose try to force closed
    if (digitalRead(LIMIT_CLOSE) == HIGH && digitalRead(LIMIT_OPEN) == HIGH) {
      Serial.println(";Magazine Slide Caught between Open/Close: Ignoring Limits and forcing opposite direction");
      deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_FORWARD, _acceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) {}
      // wait for LIMIT_OPEN switch to go low and soon as it does we 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
      Serial.println(";ERROR");
      digitalWrite(ACTIVITYLED, LOW);
      return;
    }

    // CHECK IF ITS OPEN AND IF SO CLOSE IT ELSE REPORT CLOSED
    if (digitalRead(LIMIT_OPEN) == LOW) {
      deltaprintr_motor.drive(_speed, deltaprintr_motor.DIRECTION_BACKWARD, _acceleration);
      while (digitalRead(LIMIT_CLOSE) == HIGH) { } 
      deltaprintr_motor.breakdown(0, 0); // Default is Hard Stop (should I allow this to be overridden???)
    }
      // Now let user know we're open and things are OK.
      Serial.println(";Nozzle Closed");
      digitalWrite(ACTIVITYLED, LOW); // turn LED off to show we're out of the routine
}             

void setup() {

  // Call to start GCODE Object
  GCode.begin(115200, ">"); //responce => ok, rs or !!

  // Setup our INPUT/OUTPUTS
  pinMode(ACTIVITYLED, OUTPUT);
  pinMode(LIMIT_CLOSE, INPUT_PULLUP);
  pinMode(LIMIT_OPEN, INPUT_PULLUP);

  // CREATE BUTTONS
  open_button.attach( OPEN_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  close_button.attach( CLOSE_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  
  // DEBOUNCE INTERVAL IN MILLISECONDS
  open_button.interval(5);
  close_button.interval(5); 
  
  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  open_button.setPressedState(LOW);
  close_button.setPressedState(LOW); 

  // If the ESTOP is defined create the button with a 5ms debounce and active low 
  #ifdef __ENABLE_ESTOP_SWITCH__
    estop_button.attach( ESTOP_SWITCH ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    estop_button.interval(5);
    estop_button.setPressedState(LOW);
  #endif

  // If the INA219 is defined set it up
  #ifdef __ENABLE_INA219_
    if (!ina219.begin()) {
      Serial.println("Failed to find INA219 chip");
      while (1) {
        delay(10);
      }
    }
  #endif
}

void loop() {

  // see if there is a GCODE command available
  GCode.available();

  // get button info
  open_button.update();
  close_button.update();

  // if ESTOP is defined get button and if pressed stop motor
  #ifdef __ENABLE_ESTOP_SWITCH__
    estop_button.update();
    if (estop_button.pressed()) {
      deltaprintr_motor.breakdown(0, 0);
      Serial.println(";Emergincy Stop Triggered");
    }
  #endif

  // if the open button is pressed call openNozzleMagazine
  if ( open_button.pressed() ) {
    openNozzleMagazine();
  }

  // if the close button is pressed call closeNozzleMagazine
  if (close_button.pressed()) {
    closeNozzleMagazine();
  }
}