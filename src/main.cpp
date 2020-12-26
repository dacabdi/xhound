#undef min
#undef max

#include <functional>
#include <string> 

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SparkFun_Ublox_Arduino_Library.h"

#include "power_control.h"
#include "buzzer.h"
#include "display.h"
#include "gps_config.h"
#include "eulerAngl.h"
#include "correction.h"
#include "eavesdrop.h"

#define MONITOR_SERIAL_BAUD 115200
#define EAVESDROP_SERIAL_BAUD 9600

#define BTINTPIN 0
#define ONOFFPIN 1
#define POWERPIN 2
#define BUZZERPIN 3

using namespace GNSS_RTK_ROVER;

PeriferalPowerController gps_bt_dp_power(POWERPIN);
Buzzer buzzer(BUZZERPIN);
DisplaySSD1306* display;

GPSConfig* gpsConfig;
Eavesdropper* eavesdropper;
SimpleEavesdropper simple_eavesdropper(Serial1);
UBXEavesdropper ubx_eavesdropper(Serial1);

volatile bool btConnected = false;
volatile bool btDueReset = false;

void btIntHandler()
{
    btConnected = !btConnected;
    btDueReset = !btConnected;
    digitalWrite(LED_BUILTIN, btConnected ? HIGH : LOW);
}

int CarrierSolutionType = 0;

void setup()
{
	Wire.begin();
	Serial.begin(MONITOR_SERIAL_BAUD);
	Serial1.begin(EAVESDROP_SERIAL_BAUD);
	delay(2000);
	Serial.println("UARTS & I2C Initialized...");

    display = new DisplaySSD1306(
        [&](){ // onConnected
            Serial.println("Display connected");
        },
        [&](){ // onTryingConnection
            Serial.println("Display not connected. Trying...");
        });

    Serial.println("Setting up power control");
    CPUPowerController::setup(ONOFFPIN,
        [&](){ // onWake
            buzzer.buzzPowerOn();
            // Turn on periferics
            gps_bt_dp_power.turnOn();
            delay(500);

            // Reinitialize periferics
            display->initialize();
            gpsConfig->initialize();

            //digitalWrite(LED_BUILTIN, HIGH);
            display->printTextInRect("Awake 789ABCDEF!");
        },
        [&](){ // onSleep
            buzzer.buzzPowerOff();
            Serial.println("Sleeping...");
            display->printTextInRect("Sleeping...");
            delay(500);

            //digitalWrite(LED_BUILTIN, LOW);
            gps_bt_dp_power.turnOff();
        });

    Serial.println("Setting up GPS config");
    gpsConfig = new GPSConfig(EAVESDROP_SERIAL_BAUD,
        [](){ // onConnected
            Serial.println("Ublox GNSS connected");
        },
        [](){ // onTryingConnection
            Serial.println("Ublox GNSS not connected. Trying...");
        },
        [](){ // onReset
            Serial.println("Ublox GNSS Reseted");
        },
        [&](){ // onFixedNMEA
            Serial.println("Using NMEA");
            eavesdropper = &simple_eavesdropper;
        },
        [&](){ // onFixedUBX
            Serial.println("Using UBX");
            // TODO temporary pass through, replace with: `eavesdropper = &ubx_eavesdropper;`
            eavesdropper = &simple_eavesdropper;
        });

    Serial.println("Attaching BT disconnect interrupt...");
    pinMode(BTINTPIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(BTINTPIN), btIntHandler, CHANGE);

    Serial.println("Finished Setup");
	delay(2000);
}

void loop()
{
    if(btDueReset)
    {
        Serial.println("BT disconnected, defaulting GNSS config");
        gpsConfig->initialize();
        btDueReset = false;
    }

    CPUPowerController::checkForSleep();
    //gpsConfig->checkForStatus();
	eavesdropper->eavesdrop();
}

//----------------------Functions----------------------------------;

// void displayGpsData()
// {
//   CarrierSolutionType = gpsConfig.getCarrierSolutionType();
//   // First, let's collect the position data
//   int32_t latitude = gpsConfig.getHighResLatitude();
//   int8_t latitudeHp = gpsConfig.getHighResLatitudeHp();
//   int32_t longitude = gpsConfig.getHighResLongitude();
//   int8_t longitudeHp = gpsConfig.getHighResLongitudeHp();

//   // Defines storage for the lat and lon units integer and fractional parts
//   int32_t lat_int; // Integer part of the latitude in degrees
//   int32_t lat_frac; // Fractional part of the latitude
//   int32_t lon_int; // Integer part of the longitude in degrees
//   int32_t lon_frac; // Fractional part of the longitude

//   int32_t ellipsoid = gpsConfig.getElipsoid();
//   int8_t ellipsoidHp = gpsConfig.getElipsoidHp();
//   int32_t msl = gpsConfig.getMeanSeaLevel();
//   int8_t mslHp = gpsConfig.getMeanSeaLevelHp();
//   uint32_t accuracy = gpsConfig.getHorizontalAccuracy();

//   // Calculate the latitude and longitude integer and fractional parts
//   lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
//   lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
//   lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component
//   if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
//   {
//     lat_frac = 0 - lat_frac;
//   }
//   lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
//   lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
//   lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
//   if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
//   {
//     lon_frac = 0 - lon_frac;
//   }

//   // Print the lat and lon
//   Serial.print(" -- ");
//   Serial.print("Lat (deg): ");
//   Serial.print(lat_int); // Print the integer part of the latitude
//   Serial.print(".");
//   Serial.print(lat_frac); // Print the fractional part of the latitude
//   if(lat_int > 0)
//   {
//     Serial.print(", N");
//   }
//   else
//   {
//     Serial.print(", S");
//   }
//   Serial.print(", Lon (deg): ");

  
//   Serial.print(lon_int); // Print the integer part of the latitude
//   Serial.print(".");
//   Serial.print(lon_frac); // Print the fractional part of the latitude
//     if(lon_int > 0)
//   {
//     Serial.println(", E");
//   }
//   else
//   {
//     Serial.print(", W");
//   }

//   // Now define float storage for the heights and accuracy
//   float f_ellipsoid;
//   float f_msl;
//   float f_accuracy;

//   // Calculate the height above ellipsoid in mm * 10^-1
//   f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
//   // Now convert to m
//   f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

//   // Calculate the height above mean sea level in mm * 10^-1
//   f_msl = (msl * 10) + mslHp;
//   // Now convert to m
//   f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m
//   // Convert the horizontal accuracy (mm * 10^-1) to a float
//   f_accuracy = accuracy;
//   // Now convert to m
//   f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

//   // Finally, do the printing
//   Serial.print(", Ellipsoid (m): ");
//   Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places
//   Serial.print(", Mean Sea Level(m): ");
//   Serial.print(f_msl, 4); // Print the mean sea level with 4 decimal places

//   Serial.print(", Accuracy (m): ");
//   Serial.print(f_accuracy, 4); // Print the accuracy with 4 decimal places

//   switch(CarrierSolutionType)
//   {
//     case 0:
//       Serial.println(", No Solution");
//       break;
//     case 1:
//       Serial.println(", Float RTK");
//       break;
//     case 2:
//       Serial.println(", Fix RTK");
//       break;
//   }
// }