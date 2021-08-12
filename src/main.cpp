#undef min
#undef max

#include <vector>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <cstdlib>

#include <Arduino.h>
#include <Wire.h>
#include <Timer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <FlashStorage.h>

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
#include "rec_monitor.h"
#include "schedule.h"
#include "leds.h"
#include "screens.h"
#include "views.h"
#include "views_menu.h"

// Device Info
constexpr char model[] = "XHound V1.27";
constexpr char sn[]    = "XH-00000001";
constexpr char btID[]  = "XHound";

#define MONITOR_SERIAL_BAUD 115200
#define GPS_UART1_BAUD 115200
#define GPS_UART2_BAUD 115200

#define ONOFFPIN 1
#define MAINPOWERPIN A2
#define CHARGINGPIN 7
#define BATTERYPIN A1

#define RIGHTKEYPIN 4
#define LEFTKEYPIN 5

#define PERIPHERALPOWERPIN 0

#define ONOFF_LEDPIN A3
#define BATTERY_LEDPIN A4
#define RTK_LEDPIN 3
#define DGPS_LEDPIN 2

#define BUZZERPIN A0
#define BLUETOOTHPIN 6

#define DISPLAYOFFSETX 4
#define DISPLAYOFFSETY 0

using namespace GNSS_RTK_ROVER;

PeripheralPowerController peripheralPower;

Buzzer buzzer(BUZZERPIN);

LED onOffLED(ONOFF_LEDPIN);
LED batteryLED(BATTERY_LEDPIN);
LED rtkLED(RTK_LEDPIN);
LED dgpsLED(DGPS_LEDPIN);


Canvas* display;
LogoView* logoView;

CompositeComponent* mainScreen;
DivisionLineView* mainDivisionLineView;
BatteryView* batteryView;
BTStatusView* btStatusView;
SolutionTypeView* solutionTypeView;
SIVView* sivView;
DOPView* dopView;

CompositeComponent* coordinatesScreen;
CoordinatesView* coordinatesView;

CompositeComponent* dopScreen;
DOPScreenView* dopScreenView;

CompositeComponent* baseInfoScreen;
BaseInfoView* baseInfoView;

CompositeComponent* deviceInfoScreen;
DeviceInfoView* deviceInfoView;

Schedule schedule;

void createScreens()
{
    // Main screen
    mainDivisionLineView = new DivisionLineView(display, {0, 17});
    batteryView = new BatteryView(display, {114, 1});
    btStatusView = new BTStatusView(display, {103, 0});
    solutionTypeView = new SolutionTypeView(display, {0, 0});
    sivView = new SIVView(display, {0, 22});
    dopView = new DOPView(display, {65, 22});
    mainScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    mainScreen->embed(mainDivisionLineView);
    mainScreen->embed(batteryView);
    mainScreen->embed(btStatusView);
    mainScreen->embed(solutionTypeView);
    mainScreen->embed(sivView);
    mainScreen->embed(dopView);

    // Coordinates screen
    coordinatesView = new CoordinatesView(display, {0, 0});
    coordinatesScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    coordinatesScreen->embed(coordinatesView);

    // DOP screen
    dopScreenView = new DOPScreenView(display, {0, 0});
    dopScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    dopScreen->embed(dopScreenView);

    // BaseInfo screen
    baseInfoView = new BaseInfoView(display, {0, 0});
    baseInfoScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    baseInfoScreen->embed(baseInfoView);

    // DeviceInfo screen
    deviceInfoView = new DeviceInfoView(display, {0, 0}, String(model), String(sn), String(btID));
    deviceInfoScreen = new CompositeComponent(display, {0, 0}, {32, 128}); 
    deviceInfoScreen->embed(deviceInfoView);

    ScreenManager::setup({mainScreen, coordinatesScreen, dopScreen, baseInfoScreen, deviceInfoScreen});

    pinMode(RIGHTKEYPIN, INPUT_PULLUP);
    pinMode(LEFTKEYPIN, INPUT_PULLUP);
    attachInterrupt(RIGHTKEYPIN, ScreenManager::queueNextScreen, FALLING);
    attachInterrupt(LEFTKEYPIN, ScreenManager::queuePreviousScreen, FALLING);
}

void deleteScreens()
{
    ScreenManager::stop();

    // Main screen
    delete mainScreen;
    delete mainDivisionLineView;
    delete batteryView;
    delete btStatusView;
    delete solutionTypeView;
    delete sivView;
    delete dopView;

    // Coordinates screen
    delete coordinatesScreen;
    delete coordinatesView;

    // Coordinates screen
    delete dopScreen;
    delete dopScreenView;

    // BaseInfo screen
    delete baseInfoScreen;
    delete baseInfoView;

    // DeviceInfo screen
    delete deviceInfoScreen;
    delete deviceInfoView;

    // Logo
    delete logoView;
}

void start()
{
    onOffLED.set(5);
    buzzer.buzzPowerOn();
    peripheralPower.turnOn();
    delay(1000);

    // Display and Views
    display = new DisplaySSD1306({DISPLAYOFFSETX, DISPLAYOFFSETY},
        [&](){ // onConnected
            Serial.println("Display connected");
        },
        [&](){ // onTryingConnection
            Serial.println("Display not connected. Trying...");
        });
    logoView = new LogoView(display, {0, 0});

    BatteryMonitor::start(BATTERYPIN,
        [&](float_t voltage, bool isCharging, bool batteryFull){ // onVoltageChanged
            Serial.println("onVoltageChanged");
            auto percentage = batteryFull ? 100 : BatteryPercentageProvider::getBatteryPercentage(voltage, isCharging);
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
        },
        [&](){ // onBatteryFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255);
        },
        [&](){ // onBatteryNotFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255, 500);
        },
        [&](){ //onBatteryDead
            CPUPowerController::turnOff();
        });
    Serial.println("Finished setting up battery monitor");

    BluetoothMonitor::start(BLUETOOTHPIN,
        [&](){ // onConnected
            Serial.println("Bluetooth connected");
            btStatusView->setStatus(true);
            btStatusView->draw();
            buzzer.buzzBTConnected();

            GPSConfig::configureDefault();
            GPSConfig::WakeUp();
        },
        [&](){ // onDisconnected
            Serial.println("Bluetooth disconnected");
            btStatusView->setStatus(false);
            btStatusView->draw();
            buzzer.buzzBTDisconnected();

            GPSConfig::configureDefault();
            GPSConfig::Sleep();

        });
    Serial.println("Finished setting up bluetooth monitor");

    GPSConfig::start(GPS_UART1_BAUD, GPS_UART2_BAUD,
        [&](){ // onConnected
            Serial.println("GNSS connected");
            GPSConfig::configureDefault();
            GPSConfig::Sleep();
        },
        [&](){ // onTryingConnection
            Serial.println("GNSS not connected. Trying...");
        },
        [&](GPSConfig::GPSData& data){ // update
        Serial.print("Solution type: ");
            switch(data.solType)
            {
                case GPSConfig::GnssOff:
                    solutionTypeView->setStatus(SolutionTypeView::GnssOff);
                    Serial.println("GNSS Off");
                    break;
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

            if(data.solType < GPSConfig::NoFix)
            {
                rtkLED.set(0);
                dgpsLED.set(255);
            }
            else if(data.solType < GPSConfig::FloatRTK)
            {
                rtkLED.set(0);
                dgpsLED.set(255, 500);
            }
            else if(data.solType < GPSConfig::FixedRTK)
            {
                rtkLED.set(255, 500);
                dgpsLED.set(0);
            }
            else
            {
                rtkLED.set(255);
                dgpsLED.set(0);
            }

            auto sel = data.solType == GPSConfig::GnssOff ? true : false;
            coordinatesView->setPowerSaving(sel);
            dopScreenView->setPowerSaving(sel);
            sivView->setPowerSaving(sel);
            dopView->setPowerSaving(sel);
            baseInfoView->setPowerSaving(sel);

            auto latlon = GPSConfig::getLatLonHRPretty();
            coordinatesView->setCoordinates(latlon.first, latlon.second, data.alt);
            coordinatesView->draw();

            dopScreenView->setDOP(data.hdop, data.vdop, data.pdop);
            dopScreenView->draw();

            sivView->setSIV(data.siv);
            sivView->draw();

            dopView->setDOP(data.pdop);
            dopView->draw();

            baseInfoView->setInfo(data.refID, data.refDistance);
            baseInfoView->draw();
        });

    logoView->draw();
    delay(5000);
    logoView->clear();

    createScreens();
}

void stop()
{
    BatteryMonitor::start(BATTERYPIN,
        [&](float_t voltage, bool isCharging, bool batteryFull){ // onPercentageChanged
        },
        [&](){ // onBatteryFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255);
        },
        [&](){ // onBatteryNotFull
            if(CPUPowerController::isCharging())
                batteryLED.set(255, 500);
        },
        [&](){ // onBatteryDead
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

    dgpsLED.set(0);
    rtkLED.set(0);

    delete display;
    delete logoView;

    deleteScreens();
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
    analogReadResolution(10);
    analogReference(AR_INTERNAL2V23);

    // I2C and UART
    Wire.begin();
	Serial.begin(MONITOR_SERIAL_BAUD);
	Serial.println("UARTS & I2C Initialized...");
    peripheralPower.setup(PERIPHERALPOWERPIN, HIGH);
    Serial.println("Setting up power control");

    CPUPowerController::setup(ONOFFPIN, MAINPOWERPIN, CHARGINGPIN,
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

    schedule.AddEvent(100, LED::refreshInstances);
    schedule.AddEvent(200, ScreenManager::refresh);
    schedule.AddEvent(500, CPUPowerController::checkOnOffStatus);
    schedule.AddEvent(750, CPUPowerController::checkCharging);
    schedule.AddEvent(1000, GPSConfig::checkUbloxCallbacks);
    schedule.AddEvent(1200, BluetoothMonitor::checkStatus);
    schedule.AddEvent(1500, GPSConfig::checkStatus);
    schedule.AddEvent(5000, BatteryMonitor::checkStatus);

    Serial.println("Finished Setup");
}

void loop()
{
    GPSConfig::checkUblox();
    schedule.Update();

    uint8_t select = 0;
    if(Serial.available())
    {
        select = Serial.read() - '0';

        Serial.print("Input is: "); Serial.println(select);

        if(select < 1)
        {
            Serial.println("LeftLED: GNSS OFF");
            rtkLED.set(0);
            dgpsLED.set(255);
        }
        else if(select < 2)
        {
            Serial.println("LeftLED: NO RTK");
            rtkLED.set(0);
            dgpsLED.set(255, 500);
        }
        else if (select < 3)
        {
            Serial.println("LeftLED: FLOAT RTK");
            rtkLED.set(255, 500);
            dgpsLED.set(0);
        }
        else
        {
            Serial.println("LeftLED: FIXED RTK");
            rtkLED.set(255);
            dgpsLED.set(0);
        }
    }
}