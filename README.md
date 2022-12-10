# OTP
OpenTag Ping is an open source acoustic pinger for fish and other animals.
OTP is unique in that it incorporates an accelerometer that can be used 
to determine when an animal is vocalizing, which then triggers a coded acoustic pulse.

## Altium
PCB files for OTP tags. Each PCB project will have a PDF document containing the schematic and PCB layout. The project outputs folder contain the Gerber files needed to have the PCB manufactured.

1. OTP: Simple design with ATmega328p and LIS2DS12 accelerometer. Not recommended.
2. OTP2: Improved transmitter circuit. Not recommended.
3. OTP3: Transmitter diode changed. Not recommended.
4. OTP4: PWM moved to PD3 to take advantage of built-in PWM.
5. OTP5: Smaller board designed to match lithium battery size. Recommended.
6. OTP6: Design for custom piezoceramic where lithium battery can be placed inside piezo cylinder. Also adds pressure and depth sensor. Recommended.

## Arduino
Arduino C code for programming OTP tags. 

The accelerometer uses SPI for high speed data transfer to lower power consumption.
The accelerometer can be configured for 3 channel operation or in a special magnitude mode where the magnitude of the 3 axes is calculated on the accelerometer. Magnitude mode reduces power consumption because it reduces how much time it takes to transmit and process the accelerometer data.

OTP4 firmware works on OTP4 and OTP5

## SpecSheets
The technical specification sheets for the components used in OTP tags