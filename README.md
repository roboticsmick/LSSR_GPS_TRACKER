# LSR_GPS_TRACKER

A low cost modular GPS tracker for rocket launches. Uses a sparkfun ZOE-M8Q and Ebyte LORA module to communicate GPS coordinates with the LSR Rocket GPS Base Station with a range of 7km.

This configuration was designed to be mounted in the nose cone of my successful L1 HRP certification launch.

![LSR_GPS_Tracker](https://user-images.githubusercontent.com/70121687/173563023-977a44c3-287d-41a4-a456-b5ffb58ea424.png)

Hardware:

TEENSY 4.0
EBYTE LORA E22-900T22D (set to 915Mhz for Australian usage)
SPARKFUN UBLOX ZOE-M8Q GPS
MPM3610 5V 1.2A
KEYES BUZZER MODULE
ADXL375 ACCELEROMETER
LSR MICRO PCB KILL SWITCH
7.4V LIPO WITH JST CONNECTION
C++ (platformIO)
All PCB design and modelling was done using Fusion360.

PCB boards were printed from JLCPCB.
