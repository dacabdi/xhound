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
#include "leds.h"
#include "views.h"

#define MONITOR_SERIAL_BAUD 115200
#define GPS_UART1_BAUD 115200
#define GPS_UART2_BAUD 115200
#define BATTERY_DEAD_VOLT 3.70

#define ONOFFPIN 1
#define CHARGINGPIN 8
#define BATTERYPIN A1

#define PERIPHERALPOWERPIN 0

#define ONOFF_LEDPIN A3
#define BATTERY_LEDPIN A4
#define SPARE_RED 2
#define SPARE_BLUE 3

#define BUZZERPIN A0
#define BLUETOOTHPIN 7

using namespace GNSS_RTK_ROVER;

PeripheralPowerController peripheralPower;

Buzzer buzzer(BUZZERPIN);

LED onOffLED(ONOFF_LEDPIN);
LED batteryLED(BATTERY_LEDPIN);

Canvas* display;
LogoView* logoView;
DivisionLineView* divisionLineView;
BatteryView* batteryView;
BTStatusView* btStatusView;
SolutionTypeView* solutionTypeView;

Schedule schedule;

void start()
{
    onOffLED.set(5);
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
    divisionLineView = new DivisionLineView(display, {0, 17});
    batteryView = new BatteryView(display, {117, 1});
    btStatusView = new BTStatusView(display, {106, 0});
    solutionTypeView = new SolutionTypeView(display, {0, 0});

    BatteryMonitor::start(BATTERYPIN,
        [&](float_t voltage, uint8_t percentage){ // onPercentageChanged
            batteryView->setPercentage(percentage);
            batteryView->draw();

            if(percentage == 0)
            {
                onOffLED.set(100, 100);
            }
            else 
            {
                onOffLED.set(10);
            }

            if(voltage < BATTERY_DEAD_VOLT) 
            {
                CPUPowerController::turnOffPowerModule();
            }
        },
        [&](){ // onBatteryFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255);
        },
        [&](){ // onBatteryNotFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255, 0);
        },
        [&](){ // onBatteryZero
            buzzer.buzzBatteryZero();
        });
    Serial.println("Finished setting up battery monitor");

    BluetoothMonitor::start(BLUETOOTHPIN,
        [&](){ // onConnected
            Serial.println("Bluetooth connected");
            btStatusView->setStatus(true);
            btStatusView->draw();
            buzzer.buzzBTConnected();
        },
        [&](){ // onDisconnected
            Serial.println("Bluetooth disconnected");
            btStatusView->setStatus(false);
            btStatusView->draw();
            if(GPSConfig::getMode() == GPSConfig::Rover) 
            {
                GPSConfig::configureDefault();
            }
            buzzer.buzzBTDisconnected();
        });
    Serial.println("Finished setting up bluetooth monitor");

    GPSConfig::start(GPS_UART1_BAUD, GPS_UART2_BAUD,
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
                    solutionTypeView->setStatus(SolutionTypeView::NoFix);
                    Serial.println("No Fix");
                    break;
                case GPSConfig::TwoDFix:
                    solutionTypeView->setStatus(SolutionTypeView::TwoDFix);
                    Serial.println("2D Fix");
                    break;
                case GPSConfig::ThreeDFix:
                    solutionTypeView->setStatus(SolutionTypeView::ThreeDFix);
                    Serial.println("3D Fix");
                    break;
                case GPSConfig::TimeFix:
                    solutionTypeView->setStatus(SolutionTypeView::TimeFix);
                    Serial.println("Time Fix");
                    break;
                case GPSConfig::DGPS:
                    solutionTypeView->setStatus(SolutionTypeView::DGPS);
                    Serial.println("DGPS");
                    break;
                case GPSConfig::FloatRTK:
                    solutionTypeView->setStatus(SolutionTypeView::FloatRTK);
                    Serial.println("Float RTK");
                    break;
                case GPSConfig::FixedRTK:
                    solutionTypeView->setStatus(SolutionTypeView::FixedRTK);
                    Serial.println("Fixed RTK");
                    break;
                default:
                    Serial.println("Unknown");
            }
            solutionTypeView->draw();
        },
        [&](GPSConfig::Mode mode){

        });

    logoView->draw();
    delay(3000);
    logoView->clear();

    divisionLineView->draw();
    batteryView->draw();
    btStatusView->draw();
    solutionTypeView->draw();
}

void stop()
{
    BatteryMonitor::start(BATTERYPIN,
        [&](float_t voltage, uint8_t percentage){ // onPercentageChanged
        },
        [&](){ // onBatteryFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255);
        },
        [&](){ // onBatteryNotFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255, 500);
        },
        [&](){ // onBatteryZero
        });
    BluetoothMonitor::stop();
    GPSConfig::stop();
    peripheralPower.turnOff();
    buzzer.buzzPowerOff();
    
    int counter = 2;
    while(counter--)
    {
        onOffLED.set(0);
        delay(200);
        onOffLED.set(10);
        delay(200);
    }
    onOffLED.set(0);

    delete display;
    delete logoView;
    delete divisionLineView;
    delete batteryView;
    delete btStatusView;
    delete solutionTypeView;
}

void externalPowerConnected()
{
    buzzer.buzzCharging();
    BatteryMonitor::setChargingState(true);
    if(BatteryMonitor::isBatteryFull())
        batteryLED.set(255);
    else
        batteryLED.set(255, 500);
}

void externalPowerDisconnected()
{
    buzzer.buzzNoCharging();
    BatteryMonitor::setChargingState(false);
    batteryLED.set(0);
}

void setup()
{
    delay(3000);
    // I2C and UART 
    Wire.begin();
	Serial.begin(MONITOR_SERIAL_BAUD);
	delay(2000);
	Serial.println("UARTS & I2C Initialized...");

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

    schedule.AddEvent(500, CPUPowerController::checkOnOffStatus);
    schedule.AddEvent(2000, CPUPowerController::checkCharging);
    schedule.AddEvent(100, LED::refreshInstances);
    schedule.AddEvent(5000, BatteryMonitor::checkStatus);
    schedule.AddEvent(1000, BluetoothMonitor::checkStatus);
    schedule.AddEvent(1000, GPSConfig::checkStatus);

    Serial.println("Finished Setup");
	delay(2000);
}

void loop()
{
    schedule.Update();
}