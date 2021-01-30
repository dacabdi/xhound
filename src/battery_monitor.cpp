#include <functional>
#include <map>

#include "battery_monitor.h"

namespace GNSS_RTK_ROVER
{
    uint8_t BatteryMonitor::percentage;
	float_t BatteryMonitor::voltage;
	uint8_t BatteryMonitor::batteryPin;
	std::map<uint16_t, uint8_t> BatteryMonitor::voltToPercMap;
	std::function<void(float_t, uint8_t)> BatteryMonitor::onPercentageChanged;

    void BatteryMonitor::setup(std::function<void(float_t, uint8_t)> p_onPercentageChanged)
    { 
        onPercentageChanged = p_onPercentageChanged;
        initializeVoltToPercMap();
    }

    void BatteryMonitor::checkStatus()
    {
        readAndCalculateVoltage();
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
        auto reading = analogRead(batteryPin);
        voltage = (VOLTAGEDIVIDER * AREF * reading) / 1023;
        return voltage;
    }

	uint8_t BatteryMonitor::calculatePercentage()
    {
        int voltDiscrete = voltage * 100;
        return voltToPercMap[voltDiscrete];
    }

    void BatteryMonitor::initializeVoltToPercMap()
    {
        for(int i = 250; i <= 270; i++)
        {
            voltToPercMap[i] = 0;
        }
        for(int i = 271; i <= 330; i++)
        {
            voltToPercMap[i] = 25;
        }
        for(int i = 331; i <= 380; i++)
        {
            voltToPercMap[i] = 50;
        }
        for(int i = 381; i <= 400; i++)
        {
            voltToPercMap[i] = 75;            
        }
        for(int i = 401; i <= 450; i++)
        {
            voltToPercMap[i] = 100;
        }
    }
}