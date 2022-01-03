# Openpnp Nozzle Magazine Firmware

Deltaprintr Nozzle Magazine Firmware for ATSAMD21E18 + 8871 Motor Driver + INA219 Highside Current Sense 
Current sensing is to allow tuning for optimal speed

## Features:

- Single Board design (see Nozzle Magazine Hardware)
- USB Micro Serial Port @ 115200 Baud
  - Optional Support for
  - Open/Close Buttons
  - E-STOP Button
  - NEOPIXEL for Status (Red/Green) and Blue for Communications
  - Configurable for Large(20) or Small(10) Nozzle Magazine    
- Can be built using a Adafruit Trinket + DRV8871 + INA210

## GCODEs Supported 

- M114 to report Open/Close
- M119 to report limit switch status
- M115 to report Firmware Data  
- M111 to enable Debugging
- M220 to Set Speed, Acceleration, Braking and Timeout
- M804 to Open the Nozzle Holder
- M805 to Close the Nozzle Holder

Default Init String is **M220 S220 A0 B0 T2000**






