#undef min
#undef max

#include <vector>
#include <functional>
#include <map>
#include <string> 

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
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
#define SDCARD_CHIP_SELECT SDCARD_SS_PIN

//Input Pins//
//-------------------------------------------------------------------------------------------------------//
//Connected to On/Off Pushbutton and On/Off Key off the Power Module in parallel.
#define ONOFFPIN 1
//Connected to the LiPo Battery to monitor the Voltage.
#define BATTERYPIN A1
//Connected to the Power Module Input so we can monitor if there is External Power.
#define CHARGINGPIN 8 
//Connected to the Bluetooth Module STATE Pin so we can monitor the state off the Bluetooth Connection. 
#define BLUETOOTHPIN 7
//-------------------------------------------------------------------------------------------------------//

//Output Pins
//-------------------------------------------------------------------------------------------------------//
//Connected to the input of the Buzzer circuit for audible indications.
#define BUZZERPIN A0
//Connected to the anode of the LED1 to indicate the peripherals power state. Also if flashing the Battery is in 0%.
#define ONOFF_LEDPIN A3
//Connected to the anode of the LED2 to indicate Charging state and external power presence.
#define BATTERY_LEDPIN A4
//
#define SPARE_RED 2
//
#define SPARE_BLUE 3
//Connected to the Gate of Q1 to power on/off the peripherals.
#define PERIPHERALPOWERPIN 0
//-------------------------------------------------------------------------------------------------------//

using namespace GNSS_RTK_ROVER;

Sd2Card sdCard;
SdVolume sdCardVolume;
SdFile sdCardRoot;
File currentFile;

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

void initSDCard()
{
    String sdCardType = "Unknown";
    String sdCardSize = "SD Card Size: ";
    String sdCardVolumeType = "SD Card Volume Type: FAT ";
    uint32_t VolumeSize = 0;

    if (!sdCard.init(SPI_HALF_SPEED,SDCARD_CHIP_SELECT))
    {
        Serial.println("SDCard BAD or NOT PRESENT");
    }
    else
    {
        Serial.println("SD Card Initialized...");
        switch (sdCard.type())
        {
            case SD_CARD_TYPE_SD1:
            sdCardType = "SD1";
            break;
            case SD_CARD_TYPE_SD2:
            sdCardType = "SD2";
            break;
            case SD_CARD_TYPE_SDHC:
            sdCardType = "SDHC";
            break;
        }
        Serial.print("SD Card type: "); Serial.println(sdCardType);
        if(!sdCardVolume.init(sdCard))
        {
            Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        }
        else
        {
            sdCardSize = sdCardSize + sdCard.cardSize()/(2048);
            Serial.println(sdCardSize + " MB");
            sdCardVolumeType = sdCardVolumeType + sdCardVolume.fatType();
            Serial.println(sdCardVolumeType);
        }
    }
}

void sdCardFilesList()
{
    Serial.println("-------------------------------------------------------");
    sdCardRoot.openRoot(sdCardVolume);
    sdCardRoot.ls(LS_R | LS_DATE | LS_SIZE, 5);
    Serial.println("-------------------------------------------------------");
    sdCardRoot.close();
}

void sdCardFileGet()
{
    if(!SD.begin(SDCARD_CHIP_SELECT))
    {
        Serial.println("SDCard BAD or NOT PRESENT");
    }
    else
    {
        SD.remove()
    }
}

void readUserInput()
{
    int userInput = 255;
    while(Serial.available())
    {
        userInput = Serial.read();
    }
    if(userInput != 255)
    {
        switch (userInput)
        {
            case 49:
                sdCardFilesList();
                break;
            case 50:
                sdCardFileGet();
                break;
            default:
                break;
        }
    }
}

void setup()
{
    delay(3000);
    Wire.begin(); //I2C Init
	Serial.begin(MONITOR_SERIAL_BAUD); //Serial Init
	delay(2000);
	Serial.println("UARTS & I2C Initialized...");
    initSDCard(); //SD Card Init

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
    Serial.println("Command Mode");
    Serial.println("Press 1 to LIST Files on SDCARD");
    Serial.println("Press 2 to GET Files on SDCARD");
}

void loop()
{
    schedule.Update();
    readUserInput();
}

