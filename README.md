# Openpnp Nozzle Magazine Firmware

Deltaprintr Nozzle Magazine Firmware for the Deltaprinr Nozzle Magazine 
![](https://23re3128oij2wuxh2nd3ndfi-wpengine.netdna-ssl.com/wp-content/uploads/2021/10/Nozzle_Magazine.png)

Uses a ATSAMD21E18 + 8871 Motor Driver to control the Nozzle Magazine
Current sensing is to allow tuning for optimal speed

## Features:

- Single Board design (see Nozzle Magazine Hardware)
- Limit Switches Supported
- JAM Recovery
- Error Reporting
- USB Micro Serial Port @ 115200 Baud
- Optional Support for (at compile time)
  - INA219 Highside Current Sense 
  - Open/Close Buttons
  - E-STOP Button
  - EEPROM Stores all settings to recall at startup time
  - NEOPIXEL for Status (Red/Green) and Blue for Communications
  - Configurable for Large(20) or Small(10) Nozzle Magazine    
- Can be built using a Adafruit Trinket + DRV8871 + INA210

## GCODEs Supported 

- M111 to enable Debugging
- M114 to report Open/Close
- M115 to report Firmware Data 
- M119 to report limit switch status
- M220 to Set Speed, Acceleration, Braking and Timeout
- M804 to Open the Nozzle Holder
- M805 to Close the Nozzle Holder

Default Init String is **M220 S220 A0 B0 T2000**



