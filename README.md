# OTP
OpenTag Ping is an open source acoustic pinger for fish and other animals.
OTP is unique in that it incorporates an accelerometer that can be used 
to determine when an animal is vocalizing, which then triggers a coded acoustic pulse.

## Altium
PCB files for OTP tags
OTP: Simple design with ATmega328p and LIS2DS12 accelerometer. Not recommended.
OTP2: Improved transmitter circuit. Not recommended.
OTP3: Transmitter diode changed. Not recommended.
OTP4: PWM moved to PD3 to take advantage of built-in PDM.
OTP5: Smaller board designed to match lithium battery size. Recommended
OTP6: Design for custom piezoceramic where lithium battery can be placed inside piezo cylinder. Also adds pressure and depth sensor. Recommended.

## Arduino
Arduino C code for programming OTP tags

## SpecSheets
The technical specification sheets for the components used in OTP tags