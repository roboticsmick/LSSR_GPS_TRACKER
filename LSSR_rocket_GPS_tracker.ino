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

#define PIN_M0 5
#define PIN_M1 6
#define PIN_AX 9

#define LEFT_MID 1
#define LED_LEFT 0

// Connect to any of the Teensy Serial ports
#define ESerial Serial2
SFE_UBLOX_GNSS myGNSS;
// create the transceiver object, passing in the serial and pins
EBYTE Transceiver(&ESerial, PIN_M0, PIN_M1, PIN_AX);

int Chan;
unsigned long lastTime = 0; // Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long startTime = 0; // Used to calc the actual update rate.
int GNSSCHECK = 0; // Counter for u-blox errors

// GPS packet
struct DATA {
  long lat;
  long lon;
  long alt;
  byte SIV;
};

DATA GPSData;

void setup() {
  
  // Init serial
  Serial.begin(115200);
  
  // Updated sparkfun libraries for Wire1.begin()
  Wire1.begin();
  Wire1.setClock(400000); // Increase I2C clock speed to 400kHz

  // Start transceiver Serial2
  // Baud rate set to 9600
  ESerial.begin(9600);
    // Init pinModes for transciever
  Transceiver.init();
  // Start 
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    GNSSCHECK++;
  }
  if (myGNSS.setI2COutput(COM_TYPE_UBX) == false) //Set the I2C port to output UBX only (turn off NMEA noise)
  {
    GNSSCHECK++;
  }
  if (myGNSS.setHighPrecisionMode(true) == false) // Enable High Precision Mode - include extra decimal places in the GGA messages
  {
    GNSSCHECK++;
  }
  if (myGNSS.setNavigationFrequency(2) == false) //Set output to 5 times a second
  {
    GNSSCHECK++;
  }
  if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to AIRBORNE4g
  {
    GNSSCHECK++;
  }
  pinMode(LEFT_MID, OUTPUT); // Set 0 as LED output
  pinMode(LED_LEFT, OUTPUT); // Set 1 as LED output
  if (GNSSCHECK == 0) // Set the dynamic model to AIRBORNE4g
  {
    digitalWrite(LEFT_MID, HIGH);
  }

}

void loop() {

  if (millis() - lastTime > 500)
  {
    lastTime = millis(); //Update the timer
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
    if (GPSData.SIV >= 3) {
      digitalWrite(LED_LEFT, HIGH);
    } 
    else {
      digitalWrite(LED_LEFT, LOW);
    }
    
    // Comment out serial when not testing
    // Convert to googlemaps coordinates 
    // Serial.print(GPSData.lat / 10000000., 7);
    // Serial.print(F(", "));
    // Serial.print(GPSData.lon / 10000000., 7);
    // Serial.print(F(", "));
    // Serial.print(GPSData.alt / 1000., 2);
    // Serial.print(F(", "));
    // Serial.print(GPSData.SIV);
    // Serial.println();

    myGNSS.flushPVT();
    //
    Transceiver.SendStruct(&GPSData, sizeof(GPSData));
  }
}
