# Openpnp Nozzle Magazine Firmware


Deltaprinr Nozzle Magazine Firmware for the [Deltaprintr Nozzle Magazine](https://www.deltaprintr.com/product/nozzle-magazine/)
![](https://23re3128oij2wuxh2nd3ndfi-wpengine.netdna-ssl.com/wp-content/uploads/2021/10/Nozzle_Magazine.png)

Uses a ATSAMD21 + DRV8871 Motor Driver to control the Nozzle Magazine

## Features:

- Single Board design (see Nozzle Magazine Hardware)
- Can be built using a Adafruit Trinket + DRV8871 + INA210 or any ATSAMD21 Board
- Limit Switches Supported
- JAM Recovery
- Error Reporting
- USB Micro Serial Port @ 115200 Baud
- Optional Support for (at compile time)
  - INA219 Highside Current Sense (tune the magazine with **M111 L3**)
  - Open/Close Buttons
  - E-STOP Button
  - EEPROM Stores all settings to recall at startup time
  - NEOPIXEL for Status (Red/Green) and Blue for Communications
  - Configurable for Large(20) or Small(10) Nozzle Magazine    


## GCODEs Supported 

- G4   S[VALUE] P[VALUE]
- M43  P[VALUE] F[VALUE] S[VALUE]  (Note Due to the GCODE Driver F[VALUE] is used instead of M[VALUE])
- M111 P[VALUE] L[VALUE] to enable Debugging 
- M112 P[VALUE] to E-STOP
- M114 to report Open/Close
- M115 to report Firmware Data 
- M119 to report limit switch status
- M220 to Set Speed, Acceleration, Braking and Timeout
- M303 to Autotune **(not yet implemented)**
- M804 to Open the Nozzle Holder
- M805 to Close the Nozzle Holder

Default Init String is **M220 S220 A0 B0 T2000 D128**



