#include <functional>
#include <map>

#include "battery_monitor.h"

namespace GNSS_RTK_ROVER
{
    bool BatteryMonitor::initialized = false;
    int8_t BatteryMonitor::percentage;
	float_t BatteryMonitor::voltage;
	int8_t BatteryMonitor::batteryPin;
	std::map<uint16_t, uint8_t> BatteryMonitor::voltToPercMap;
	std::function<void(float_t, uint8_t)> BatteryMonitor::onPercentageChanged;

    void BatteryMonitor::start(uint8_t p_batteryPin, std::function<void(float_t, uint8_t)> p_onPercentageChanged)
    { 
        initialized = true;
        percentage = -1;
        batteryPin = p_batteryPin;
        pinMode(batteryPin, INPUT);

        onPercentageChanged = p_onPercentageChanged;
    }

    void BatteryMonitor::stop()
    { 
        initialized = false;
    }

    void BatteryMonitor::checkStatus()
    {
        if(!initialized)  // Skip if not setup yet
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
            return 66;
        }
        else if(voltDiscrete > 370)
        {
            return 33;
        }
        else
        {
            return 0;
        }
    }
}