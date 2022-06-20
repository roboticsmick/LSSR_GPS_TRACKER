# LSR_GPS_TRACKER

A low cost GPS rocket tracker. Uses a Teensy 4.0, Sparkfun ZOE-M8Q, and Ebytes LORA module to transmit GPS coordinates and altitude data to my LSR GPS Rocket Tracking Base Station. Estimated range is 7km.

This was designed and built to mount in the nose cone for my HPR L1 certification launch. Tracking worked perfectly and I was able to successfully transmit the rockets GPS coordinates and altitude down to the base station.

![LSR_GPS_Tracker](https://user-images.githubusercontent.com/70121687/173563023-977a44c3-287d-41a4-a456-b5ffb58ea424.png)

Parts list:
Microcontroller – TEENSY 4.0
RF transmitter – Ebyte LORA E22-900T22D (set to 915Mhz for Australian usage)
GPS – Sparkfun UBLOX ZOE-M8Q GPS
5V Buck Converter – MPM3610 5V 1.2A
Buzzer – Keyes buzzer module
Accelerometer – ADXL375 CJMCU-375
Kill switch – LSR PCB micro kill switch
Battery – 7.4V LIPO with JST connection

Version 2 will use a Raspberry Pi Pico RP2040 to bring the overall costs of the design down.

PCB design was done using Fusion 360 electronics. Case design and animation were also done using Fusion 360.

PCB boards were printed from JLCPCB.
