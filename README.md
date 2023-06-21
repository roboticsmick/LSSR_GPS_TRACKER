# LSSR_GPS_TRACKER

A low cost GPS rocket tracker. Uses a Teensy 4.0, Sparkfun ZOE-M8Q, and Ebytes LORA module to transmit GPS coordinates and altitude data to my LSR GPS Rocket Tracking Base Station. Estimated range is 7km.

This was designed and built to mount in the nose cone for my HPR L1 certification launch. Tracking worked perfectly and I was able to successfully transmit the rockets GPS coordinates and altitude down to the base station.

![LSR_GPS_Tracker](https://user-images.githubusercontent.com/70121687/173563023-977a44c3-287d-41a4-a456-b5ffb58ea424.png)

Parts list:
* Microcontroller – TEENSY 4.0
* RF transmitter – Ebyte LORA E22-900T22D (set to 915Mhz for Australian usage)
* GPS – Sparkfun UBLOX ZOE-M8Q GPS
* 5V Buck Converter – MPM3610 5V 1.2A
* Buzzer – Keyes buzzer module
* Accelerometer – ADXL375 CJMCU-375
* Kill switch – LSR PCB micro kill switch
* Battery – 7.4V LIPO with JST connection

PCB design was done using Fusion 360 electronics. Case design and animation were also done using Fusion 360.

PCB boards were printed from JLCPCB.

Instructions:
1. Connect lipo power to board.
2. Monitor base station. 
3. When SIV is 3+, ready to launch.

Important note:
The sparkfun ZOE-M8Q library uses the wire library for the I2C reference. This needs to be updated in their library files if you use the teensy 4.0 (i.e. the GPS on the teensy uses a different I2C output knot default wire.begin), so the sparkfun wire reference needs to be updated to the specific Teensy I2C output line).
