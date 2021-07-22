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
#include <SparkFun_Ublox_Arduino_Library.h>
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
#define SPARE_RED 2
#define SPARE_BLUE 3

#define BUZZERPIN A0
#define BLUETOOTHPIN 6

#define DISPLAYOFFSETX 4
#define DISPLAYOFFSETY 0

using namespace GNSS_RTK_ROVER;

FlashStorage(tolete, int);

PeripheralPowerController peripheralPower;

Buzzer buzzer(BUZZERPIN);

LED onOffLED(ONOFF_LEDPIN);
LED batteryLED(BATTERY_LEDPIN);

Canvas* display;
LogoView* logoView;

CompositeComponent* mainScreen;
DivisionLineView* divisionLineView;
BatteryView* batteryView;
BTStatusView* btStatusView;
SolutionTypeView* solutionTypeView;
SIVView* sivView;
DOPView* dopView;

CompositeComponent* coordinatesScreen;
CoordinatesView* coordinatesView;

CompositeComponent* baseInfoScreen;
BaseInfoView* baseInfoView;

Schedule schedule;

void createScreens()
{
    // Main screen
    divisionLineView = new DivisionLineView(display, {0, 17});
    batteryView = new BatteryView(display, {114, 1});
    btStatusView = new BTStatusView(display, {103, 0});
    solutionTypeView = new SolutionTypeView(display, {0, 0});
    sivView = new SIVView(display, {0, 22});
    dopView = new DOPView(display, {65, 22});
    mainScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    mainScreen->embed(divisionLineView);
    mainScreen->embed(batteryView);
    mainScreen->embed(btStatusView);
    mainScreen->embed(solutionTypeView);
    mainScreen->embed(sivView);
    mainScreen->embed(dopView);

    // Coordinates screen
    coordinatesView = new CoordinatesView(display, {0, 0});
    coordinatesScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    coordinatesScreen->embed(coordinatesView);

    // BaseInfo screen
    baseInfoView = new BaseInfoView(display, {0, 0});
    baseInfoScreen = new CompositeComponent(display, {0, 0}, {32, 128});
    baseInfoScreen->embed(baseInfoView);

    ScreenManager::setup({mainScreen, coordinatesScreen, baseInfoScreen});

    pinMode(RIGHTKEYPIN, INPUT_PULLUP);
    pinMode(LEFTKEYPIN, INPUT_PULLUP);
    attachInterrupt(RIGHTKEYPIN, ScreenManager::nextScreen, FALLING);
    attachInterrupt(LEFTKEYPIN, ScreenManager::previousScreen, FALLING);
}

void deleteScreens()
{
    ScreenManager::stop();

    // Main screen
    delete mainScreen;
    delete divisionLineView;
    delete batteryView;
    delete btStatusView;
    delete solutionTypeView;
    delete sivView;
    delete dopView;

    // Coordinates screen
    delete coordinatesScreen;
    delete coordinatesView;

    // BaseInfo screen
    delete baseInfoScreen;
    delete baseInfoView;

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
                batteryLED.set(255, 0);
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
            sivView->setSIV(235);
            sivView->draw();
            buzzer.buzzBTConnected();
            GPSConfig::configureDefault();
            GPSConfig::WakeUp();
            coordinatesView->setPowerSaving(false);
            sivView->setPowerSaving(false);
            dopView->setPowerSaving(false);
        },
        [&](){ // onDisconnected
            Serial.println("Bluetooth disconnected");
            btStatusView->setStatus(false);
            btStatusView->draw();
            sivView->setPowerSaving(true);
            sivView->draw();
            dopView->setPowerSaving(true);
            dopView->draw();
            if(GPSConfig::getMode() == GPSConfig::Rover)
            {
                GPSConfig::configureDefault();
                GPSConfig::Sleep();
                coordinatesView->setPowerSaving(true);
                sivView->setPowerSaving(true);
                dopView->setPowerSaving(true);
            }
            buzzer.buzzBTDisconnected();
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

            coordinatesView->setCoordinates(data.lat, data.lon, data.alt);
            coordinatesView->draw();

            sivView->setSIV(data.siv);
            sivView->draw();

            dopView->setDOP(data.dop);
            dopView->draw();
        });

    logoView->draw();
    delay(5000);
    logoView->clear();

    // Menu list test
    // auto options = new MenuOptionPtr[5] {
    //     new MenuOption(display, {0, 0}, "testing 111"),
    //     new MenuOption(display, {0, 0}, "testing 222"),
    //     new MenuOption(display, {0, 0}, "testing 333"),
    //     new MenuOption(display, {0, 0}, "testing 444"),
    //     new MenuOption(display, {0, 0}, "testing 555"),
    // };

    // auto menuList = new MenuList(display, {0, 0}, options, 5);
    // menuList->draw();
    // delay(5000);
    // menuList->select(1);
    // menuList->draw();
    // delay(5000);
    // menuList->select(2);
    // menuList->draw();
    // delay(5000);
    // menuList->select(3);
    // menuList->draw();
    // delay(5000);
    // menuList->select(4);
    // menuList->draw();
    // delay(5000);
    // menuList->select(0);
    // menuList->draw();
    // delay(5000);
    // delay(10000);
    // delete menuList;

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
    delay(1000);

    auto curr_tolete = tolete.read();
    Serial.print("Tolete is: "); Serial.println(curr_tolete);
    tolete.write(8);

    // I2C and UART
    Wire.begin();
	Serial.begin(MONITOR_SERIAL_BAUD);
	delay(1000);
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

    schedule.AddEvent(500, CPUPowerController::checkOnOffStatus);
    schedule.AddEvent(2000, CPUPowerController::checkCharging);
    schedule.AddEvent(100, LED::refreshInstances);
    schedule.AddEvent(5000, BatteryMonitor::checkStatus);
    schedule.AddEvent(1000, BluetoothMonitor::checkStatus);
    schedule.AddEvent(2000, GPSConfig::checkStatus);
    schedule.AddEvent(2000, ScreenManager::refresh);

    Serial.println("Finished Setup");
	delay(1000);
}

void loop()
{
    schedule.Update();
}