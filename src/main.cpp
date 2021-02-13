#undef min
#undef max

#include <vector>
#include <functional>
#include <map>
#include <string> 

#include <Arduino.h>
#include <Wire.h>
#include <Timer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SparkFun_Ublox_Arduino_Library.h"

#include "power_control.h"
#include "buzzer.h"
#include "graphics.h"
#include "display.h"
#include "gps_config.h"
#include "eulerAngl.h"
#include "correction.h"
#include "eavesdrop.h"
#include "bitmaps.h"
#include "battery_monitor.h"
#include "schedule.h"
#include "views.h"

#define MONITOR_SERIAL_BAUD 115200
#define EAVESDROP_SERIAL_BAUD 115200

#define BATTERYPIN A1
#define BTSTATEPIN 0
#define ONOFFPIN 1
#define POWERPIN 2
#define BUZZERPIN 3
#define ROVERBASESWITCH 4

using namespace GNSS_RTK_ROVER;

PeriferalPowerController gps_bt_dp_power(POWERPIN);
Buzzer buzzer(BUZZERPIN);
DisplaySSD1306* display;

LogoView logoView(*display, Vector2D{0, 0});
BatteryView batteryView(*display, Vector2D{106, 0});
BTStatusView btStatusView(*display, Vector2D{84, 0});
DivisionLineView divisionLineView(*display, Vector2D{80, 0});
RoverStatusView roverStatusView(*display, Vector2D{0, 2});

Schedule schedule;

GPSConfig* gpsConfig;


byte ByteFromGNSS = 0;

int checkBatteryFlag = 0;
const int VoltageDivider =2;
const float aref = 3.3;
int BatteryReading = 0;
int BatteryActualLevel = 0;
int BatteryLastLevel = 150;
float BatteryVoltage = 0;

int checkBTstateFlag = 0;
bool btConnectionLastState = false;

int checkCarriesSolutionFlag = 0;
int ActualCarrierSolution = 0;
int LastCarrierSolution = 150;

bool RoverBaseModeFlagLast = true;
volatile bool RoverBaseModeFlag = false;

float mean_Accuracy = 0;
int ObservationTime = 0;
boolean svin_valid = 0;


// void ROVERBASESwitch()
// {
//     detachInterrupt(ROVERBASESWITCH);

//     if(RoverBaseModeFlag == false)
//     {
//         Serial.println("Going into BASE MODE");
//     }
//     else
//     {
//         Serial.println("Going into ROVER MODE");
//     }
//     RoverBaseModeFlag = !RoverBaseModeFlag;
// }

// void BridgeDataGNSStoBT()
// {   
//     while(Serial1.available())
//     {
//         ByteFromGNSS = Serial1.read();
//         Serial1.write(ByteFromGNSS);
//     }
// }

// void monitorPrintBattery()
// {
//     Serial.print("Battery Reading = ");
//     Serial.print(BatteryReading);
//     Serial.print(" --- ");
//     Serial.print("Battery Voltage = ");
//     Serial.print(BatteryVoltage);
//     Serial.print(" V");
//     Serial.print(" --- ");
//     Serial.print("Battery Level = ");
//     Serial.print(BatteryActualLevel);
//     Serial.println(" % ");
// }

// void checkBattery_BasePrecision()
// {    
//     BatteryReading = analogRead(BATTERYPIN);
//     BatteryVoltage = (VoltageDivider * AREF * BatteryReading) / 1023;

//     if(!RoverBaseModeFlag)
//     {
//         //Serial.print("RoverBaseModeFlag = "); Serial.println(RoverBaseModeFlag);
//         display->printBitMap(3, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//         display->printBitMap(10, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//         display->printText("VBat = ", 3, 25);
//         display->printBitMap(45, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//         display->printFloatVariable(BatteryVoltage, 45, 25);
//     }
//     else
//     {
//         //Serial.print("RoverBaseModeFlag = "); Serial.println(RoverBaseModeFlag);
//         display->printBitMap(3, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//         display->printBitMap(45, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
        
//         mean_Accuracy = gpsConfig->meanAccuracy();
//         if(mean_Accuracy > 10)
//         {
//             mean_Accuracy = 9.99;
//             display->printBitMap(3, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//             display->printBitMap(10, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//             display->printText("Accu > ", 3, 25);
//             display->printBitMap(45, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//             display->printFloatVariable(mean_Accuracy, 46, 25);
//         }
//         else
//         {
//             display->printBitMap(3, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//             display->printBitMap(10, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//             display->printText("Accu = ", 3, 25);
//             display->printBitMap(45, 25, clear_float_variable, 28, 8, BLACK); //This is to clear the area.
//             display->printFloatVariable(mean_Accuracy, 46, 25);
//         }
        
//     }
    
//     //monitorPrintBattery();

//     if(BatteryVoltage >= 4)
//     {
//         BatteryActualLevel = 100;
//         if(BatteryLastLevel != BatteryActualLevel)
//         {
//             display->printBitMap(106, 0, clear_icon, 21, 32, BLACK);
//             display->printBitMap(106, 0, battery_100, 21, 32, WHITE);
//             monitorPrintBattery();
//         }
//     }

//     if(BatteryVoltage < 4 && BatteryVoltage >= 3.8)
//     {
//         BatteryActualLevel = 75;
//         if(BatteryLastLevel != BatteryActualLevel)
//         {
//             display->printBitMap(106, 0, clear_icon, 21, 32, BLACK);
//             display->printBitMap(106, 0, battery_75, 21, 32, WHITE);
//             monitorPrintBattery();
//         }
//     }

//     if(BatteryVoltage < 3.8 && BatteryVoltage >= 3.3)
//     {
//         BatteryActualLevel = 50;
//         if(BatteryLastLevel != BatteryActualLevel)
//         {
//             display->printBitMap(106, 0, clear_icon, 21, 32, BLACK);
//             display->printBitMap(106, 0, battery_50, 21, 32, WHITE);
//             monitorPrintBattery();
//         }
//     }

//     if(BatteryVoltage < 3.3 && BatteryVoltage >= 2.9)
//     {
//         BatteryActualLevel = 25;
//         if(BatteryLastLevel != BatteryActualLevel)
//         {
//             display->printBitMap(106, 0, clear_icon, 21, 32, BLACK);
//             display->printBitMap(106, 0, battery_25, 21, 32, WHITE);
//             monitorPrintBattery();
//         }
//     }

//     if(BatteryVoltage <= 2.7)
//     {
//         BatteryActualLevel = 0;
//         if(BatteryLastLevel != BatteryActualLevel)
//         {
//             display->printBitMap(106, 0, clear_icon, 21, 32, BLACK);
//             display->printBitMap(106, 0, battery_0, 21, 32, WHITE);
//             monitorPrintBattery();
//         }
//     }
//     BatteryLastLevel = BatteryActualLevel;
// }

void checkBTState()
{
    bool btConnectionCurrentState = digitalRead(BTSTATEPIN);

    if(btConnectionCurrentState != btConnectionLastState)
    {
        if(btConnectionCurrentState)
        {
            Serial.println("BT Connected");
            buzzer.buzzBTConnected();
            btStatusView.setStatus(true);            
        }
        else
        {
            Serial.println("BT disconnected, defaulting GNSS config");
            buzzer.buzzBTDisconnected();
            btStatusView.setStatus(false);
            gpsConfig->initialize();
        }
        btConnectionLastState = btConnectionCurrentState;
    }
    btStatusView.clear();
    btStatusView.draw();
}

// void checkRoverBase()
// {
//     Serial.print("Base Status is = ");
//     Serial.println(gpsConfig->check_isBaseActivated());
//     // if (RoverBaseModeFlag != RoverBaseModeFlagLast)
//     // {
//     //     if(RoverBaseModeFlag)
//     //     {
//     //         buzzer.buzzBaseMode();
//     //         gpsConfig->configureAsBase();
//     //     }
//     //     else
//     //     {
//     //         buzzer.buzzRoverMode();
//     //         gpsConfig->configureDisableBase();
//     //         display->printBitMap(0, 2, clear_icon_big, 64, 15, BLACK);
//     //         display->printBitMap(0, 2, rover_mode, 64, 15, WHITE);
//     //     }
//     //     RoverBaseModeFlagLast = RoverBaseModeFlag;
//     //     attachInterrupt(digitalPinToInterrupt(ROVERBASESWITCH), ROVERBASESwitch, RISING);
//     // }
// }

void checkAndDisplayCarrierSolutionandBaseRoverMode() 
{
    //Serial.print("Checking Carrier Solutios and GNSS Mode"); Serial.print("RoverBaseModeFlag = "); Serial.println(RoverBaseModeFlag);
    if(!RoverBaseModeFlag) //Rover Mode.
    {
        ActualCarrierSolution = gpsConfig->getSolution();
        if(ActualCarrierSolution != LastCarrierSolution)
        {
            switch(ActualCarrierSolution)
            {
                case 0:
                    roverStatusView.setStatus(RoverStatus::DGPS);
                    Serial.println("No Solution");
                    break;
                case 1:
                    roverStatusView.setStatus(RoverStatus::FloatRTK);
                    Serial.println("Float RTK");
                    break;
                case 2:
                    roverStatusView.setStatus(RoverStatus::FixedRTK);
                    Serial.println("Fix RTK");
                    break;
            }
            LastCarrierSolution = ActualCarrierSolution;
        }
    }
    else
    {
        // display->printBitMap(0, 2, clear_icon_big, 64, 15, BLACK);
        // display->printBitMap(0, 2, base_mode, 64, 15, WHITE);
    }  
    roverStatusView.clear();
    roverStatusView.draw();
}

void setup()
{
    // I2C and UART 
    Wire.begin();
	Serial.begin(MONITOR_SERIAL_BAUD);
	delay(2000);
	Serial.println("UARTS & I2C Initialized...");

    // Pin Modes
    //pinMode(BTSTATEPIN, INPUT);
    //pinMode(ROVERBASESWITCH, INPUT_PULLUP);

    display = new DisplaySSD1306(
        [&](){ // onConnected
            Serial.println("Display connected");
        },
        [&](){ // onTryingConnection
            Serial.println("Display not connected. Trying...");
        });

    BatteryMonitor::setup(
        [&](float_t voltage, uint8_t percentage){ // onPercentageChanged
            batteryView.setPercentage(percentage);
            batteryView.draw();
        });

    Serial.println("Setting up power control");
    CPUPowerController::setup(ONOFFPIN,
        [&](){ // onWake
            buzzer.buzzPowerOn();
            // Turn on periferics
            gps_bt_dp_power.turnOn();
            delay(2000);
            // Reinitialize periferics
            display->initialize();
            gpsConfig->initialize();

            logoView.clear();
            logoView.draw();
            delay(5000);

            logoView.clear();
            display->printTextInRect("Waking Up ...");
            Serial.println("Waking Up ...");
            checkBTState();
            divisionLineView.draw();
            batteryView.draw();
            roverStatusView.draw();
            // Serial.println("Attaching Interrupt for the ROVER-BASE Switch");
            // attachInterrupt(digitalPinToInterrupt(ROVERBASESWITCH), ROVERBASESwitch, FALLING);
        },
        [&](){ // onSleep
            Serial.println("Detaching Interrupt for the ROVER-BASE Switch");
            detachInterrupt(ROVERBASESWITCH);
            buzzer.buzzPowerOff();
            Serial.println("Turning Off ...");
            display->printTextInRect("Turning Off ...");
            delay(2000);
            gps_bt_dp_power.turnOff();
            btConnectionLastState = false;
            BatteryLastLevel = 150;
            LastCarrierSolution = 150;
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
        [&](){ // onNMEA
            Serial.println("Using NMEA");
        },
        [&](){ // onUBX
            Serial.println("Using UBX");
            // TODO temporary pass through, replace with: `eavesdropper = &ubx_eavesdropper;`
        });

    schedule.AddEvent(4000, BatteryMonitor::checkStatus);
    schedule.AddEvent(1000, checkBTState);
    // schedule.AddEvent(2500, checkRoverBase);
    // schedule.AddEvent(4000, checkBattery_BasePrecision);
    schedule.AddEvent(5000, checkAndDisplayCarrierSolutionandBaseRoverMode);
        
    Serial.println("Finished Setup");
	delay(2000);
}

void loop()
{
    CPUPowerController::checkForSleep();
    schedule.Update();
    //BridgeDataGNSStoBT(); //Without eavesdropper.
    //gpsConfig->factoryReset();
    //gpsConfig->checkForStatus();
	//eavesdropper->eavesdrop(); //This eavesdropper needs to be fixed.
}

//----------------------Functions----------------------------------;

// void displayGpsData()
// {
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
//   // Now convert to mvoid checkAndDisplayCarrierSolution() 
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

//   switch(gpsConfig->getSolution())
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