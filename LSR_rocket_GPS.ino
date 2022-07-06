/*
  This code for for the transmitter
  Connections
  Module      Teensy/LORA
  M0          5
  M1          6
  Rx          # (This is the MCU Tx pin)
  Tx          # (This is the MCU Rx pin)
  Aux         9
  Vcc         5V
  Gnd         Gnd

  Module      Teensy/Sparfun ZOE-M8Q
  SDA         # 
  SCL         # 
  Vcc         3V3
  Gnd         Gnd
*/

#include "EBYTE.h"
#include <SoftwareSerial.h>
#include <Wire.h> // Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Updated sparkfun libraries for Wire1.begin()

// Connect to any of the Teensy Serial ports
#define ESerial Serial2
SFE_UBLOX_GNSS myGNSS;

#define PIN_M0 5
#define PIN_M1 6
#define PIN_AX 9

// GPS packet
struct DATA {
  long lat;
  long lon;
  long alt;
  byte SIV;
};

DATA GPSData;
int Chan;
long lastTime = 0; //Tracks the passing of 2000ms (2 seconds)

// create the transceiver object, passing in the serial and pins
EBYTE Transceiver(&ESerial, PIN_M0, PIN_M1, PIN_AX);

void setup() {
  
  // 
  Serial.begin(115200);
  
  // Updated sparkfun libraries for Wire1.begin()
  Wire1.begin();
  Wire1.setClock(400000); // Increase I2C clock speed to 400kHz

  // 
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  
  // Start transceiver Serial2
  // Baud rate set to 9600
  ESerial.begin(9600);
  // Init pinModes for transciever
  Transceiver.init();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
//  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setHighPrecisionMode(true); // Enable High Precision Mode - include extra decimal places in the GGA messages
  myGNSS.setNavigationFrequency(5); //Set output to 5 times a second
  
  byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);
  
}

void loop() {
  if (myGNSS.getPVT()) 
//  if (millis() - lastTime > 1000)
  {
    // 
    // lastTime = millis(); //Update the timer
    // Latitude (degrees * 10^-7)
    GPSData.lat = myGNSS.getLatitude();

    // Longitude (degrees * 10^-7)
    GPSData.lon = myGNSS.getLongitude();

    // Altitude (metres * 10^-3)
    GPSData.alt = myGNSS.getAltitude();
    
    // Satellites in view (SIV)
    GPSData.SIV = myGNSS.getSIV();

    // Comment out serial when not testing
    // Convert to googlemaps coordinates 
    Serial.print(GPSData.lat / 10000000., 7);
    Serial.print(F(", "));
    Serial.print(GPSData.lon / 10000000., 7);
    Serial.print(F(", "));
    Serial.print(GPSData.alt / 1000., 2);
    Serial.print(F(", "));
    Serial.print(GPSData.SIV);
    Serial.println();

    myGNSS.flushPVT();
    //
    Transceiver.SendStruct(&GPSData, sizeof(GPSData));
  }
}
