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
#include "pitches.h"
#include "buzzer.h"
#include "graphics.h"
#include "display.h"
#include "gps_config.h"
#include "eulerAngl.h"
#include "correction.h"
#include "eavesdrop.h"
#include "bitmaps.h"
#include "battery_monitor.h"
#include "bluetooth_monitor.h"
#include "schedule.h"
#include "views.h"

#define MONITOR_SERIAL_BAUD 115200
#define GPS_UART1_BAUD 115200
#define GPS_UART2_BAUD 115200

#define ONOFFPIN 1
#define CHARGINGPIN 8
#define BATTERYPIN A1

#define PERIPHERALPOWERPIN 0

#define ONOFFLED A3
#define SPARE_LEFT A4
#define SPARE_RIGHT 2
#define EXTERNALPOWERLED 3

#define BUZZERPIN A0
#define BLUETOOTHPIN 7

using namespace GNSS_RTK_ROVER;

PeripheralPowerController peripheralPower;

Buzzer buzzer(BUZZERPIN);

Canvas* display;
LogoView* logoView;
DivisionLineView* divisionLineView;
BatteryView* batteryView;
BTStatusView* btStatusView;

Schedule schedule;

void start()
{
    analogWrite(ONOFFLED, 25);
    buzzer.buzzPowerOn();

    peripheralPower.turnOn();
    delay(1000);

    // Display and Views
    display = new DisplaySSD1306(
        [&](){ // onConnected
            Serial.println("Display connected");
        },
        [&](){ // onTryingConnection
            Serial.println("Display not connected. Trying...");
        });
    logoView = new LogoView(display, {0, 0});
    divisionLineView = new DivisionLineView(display, {80, 0});
    batteryView = new BatteryView(display, {106, 0});
    btStatusView = new BTStatusView(display, {84, 0});

    BatteryMonitor::setup(BATTERYPIN,
        [&](float_t voltage, uint8_t percentage){
            batteryView->setPercentage(percentage);
            batteryView->draw();
        });
    Serial.println("Finished setting up battery monitor");

    BluetoothMonitor::setup(BLUETOOTHPIN,
        [&](){ // onConnected
            Serial.println("Bluetooth connected");
            btStatusView->setStatus(true);
            btStatusView->draw();
        },
        [&](){ // onDisconnected
            Serial.println("Bluetooth disconnected");
            btStatusView->setStatus(false);
            btStatusView->draw();
        });
    Serial.println("Finished setting up bluetooth monitor");

    GPSConfig::setup(GPS_UART1_BAUD, GPS_UART2_BAUD,
        [&](){ // onConnected
            Serial.println("GPS connected");
        },
        [&](){ // onTryingConnection
            Serial.println("GPS not connected. Trying...");
        },
        [&](GPSConfig::SolutionType solutionType){
            Serial.print("Solution type: ");
            switch(solutionType) 
            {
                case GPSConfig::NoFix:
                    Serial.println("No Fix");
                    break;
                case GPSConfig::TwoD:
                    Serial.println("2D");
                    break;
                case GPSConfig::ThreeD:
                    Serial.println("3D");
                    break;
                case GPSConfig::GNSS:
                    Serial.println("GNSS");
                    break;
                case GPSConfig::DGPS:
                    Serial.println("DGPS");
                    break;
                case GPSConfig::FloatRTK:
                    Serial.println("Float RTK");
                    break;
                case GPSConfig::FixedRTK:
                    Serial.println("Fixed RTK");
                    break;
                default:
                    Serial.println("Unknown");
            }
        },
        [&](GPSConfig::Mode mode){

        });

    logoView->draw();
    delay(3000);
    logoView->clear();

    divisionLineView->draw();
    batteryView->draw();
    btStatusView->draw();
}

void stop()
{
    buzzer.buzzPowerOff();
    analogWrite(ONOFFLED, 0);
}

void externalPowerConnected()
{
    buzzer.buzzCharging();
    analogWrite(EXTERNALPOWERLED, 255);
}

void externalPowerDisconnected()
{
    buzzer.buzzNoCharging();
    analogWrite(EXTERNALPOWERLED, 0);
}

void setup()
{
    delay(3000);
    // I2C and UART 
    Wire.begin();
	Serial.begin(MONITOR_SERIAL_BAUD);
	delay(2000);
	Serial.println("UARTS & I2C Initialized...");

    // Pin Modes
    pinMode(BUZZERPIN, OUTPUT);
    pinMode(ONOFFLED, OUTPUT); // TODO: Make LED controller class
    pinMode(EXTERNALPOWERLED, OUTPUT);

    peripheralPower.setup(PERIPHERALPOWERPIN, HIGH);

    Serial.println("Setting up power control");
    CPUPowerController::setup(ONOFFPIN, CHARGINGPIN,
        [&](bool onOffState){ // onTurnOnOff
            if(onOffState)
            {
                Serial.println("Turned On");
                start();
            }
            else
            {
                Serial.println("Turned Off");
                stop();        
            }   
        },
        [&](bool chargingState){
            if(chargingState) 
            {
                Serial.println("External Power Connected");
                externalPowerConnected();
            }
            else
            {
                Serial.println("External Power Disconnected");
                externalPowerDisconnected();
            }
        });

    schedule.AddEvent(1000, CPUPowerController::checkOnOffStatus);
    schedule.AddEvent(1000, CPUPowerController::checkCharging);
    schedule.AddEvent(1000, BatteryMonitor::checkStatus);
    schedule.AddEvent(1000, BluetoothMonitor::checkStatus);
    schedule.AddEvent(1000, GPSConfig::checkStatus);

    Serial.println("Finished Setup");
	delay(2000);
}

void loop()
{
    schedule.Update();
}