#include <functional>
#include <map>

#include "battery_monitor.h"

namespace GNSS_RTK_ROVER
{
    bool BatteryMonitor::initialized = false;
    uint8_t BatteryMonitor::percentage;
	float_t BatteryMonitor::voltage;
	int8_t BatteryMonitor::batteryPin;
	std::map<uint16_t, uint8_t> BatteryMonitor::voltToPercMap;
	std::function<void(float_t, uint8_t)> BatteryMonitor::onPercentageChanged;

    void BatteryMonitor::setup(uint8_t p_batteryPin, std::function<void(float_t, uint8_t)> p_onPercentageChanged)
    { 
        initialized = true;
        batteryPin = p_batteryPin;
        pinMode(batteryPin, INPUT);

        onPercentageChanged = p_onPercentageChanged;
    }

    void BatteryMonitor::checkStatus()
    {
        if(!initialized) // Skip if not setup yet
            return;

        readAndCalculateVoltage();
        //Serial.print("Battery Voltage = "); Serial.print(voltage); Serial.println(" V");
        auto new_percentage = calculatePercentage();
        if(new_percentage != percentage)
        {
            percentage = new_percentage;
            onPercentageChanged(voltage, percentage);
        }
    }

    float_t BatteryMonitor::getVoltage()
    {
        return voltage;
    }

    uint8_t BatteryMonitor::getDiscretePercentage()
    {
        return percentage;
    }

    float_t BatteryMonitor::readAndCalculateVoltage()
    {
        auto voltageReading = 0;
        for (int i = 1; i <= 30; i++)
        {
            voltageReading = (voltageReading + analogRead(batteryPin) / 30);
        }
        voltage = (VOLTAGEDIVIDER * AREF * voltageReading)/1024;
        return voltage;
    }

	uint8_t BatteryMonitor::calculatePercentage()
    {
        int voltDiscrete = voltage * 100;
        if(voltDiscrete > 400)
        {
            return 100;
        }
        else if(voltDiscrete > 380)
        {
            return 75;
        }
        else if(voltDiscrete > 370)
        {
            return 50;
        }
        else if(voltDiscrete > 360)
        {
            return 25;
        }
        else
        {
            return 0;
        }
    }

    void BatteryMonitor::initializeVoltToPercMap()
    {
        Serial.println("intializing volt map");
        for(uint16_t i = 250; i <= 270; i++)
        {
            voltToPercMap.insert(std::pair<uint16_t, uint8_t>(i, 0));
        }
        for(uint16_t i = 271; i <= 330; i++)
        {
            Serial.print("Inserting in map: ");
            Serial.println(i);
            voltToPercMap.insert(std::pair<uint16_t, uint8_t>(i, 25));
        }
        for(uint16_t i = 331; i <= 380; i++)
        {
            Serial.print("Inserting in map: ");
            Serial.println(i);
            voltToPercMap.insert(std::pair<uint16_t, uint8_t>(i, 50));
        }
        for(uint16_t i = 381; i <= 400; i++)
        {
            Serial.print("Inserting in map: ");
            Serial.println(i);
            voltToPercMap.insert(std::pair<uint16_t, uint8_t>(i, 75));      
        }
        for(uint16_t i = 401; i <= 450; i++)
        {
            Serial.print("Inserting in map: ");
            Serial.println(i);
            voltToPercMap.insert(std::pair<uint16_t, uint8_t>(i, 100));
        }
        Serial.println("finished volt map");
    }
}