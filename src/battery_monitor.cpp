#include <functional>
#include <map>

#include "battery_monitor.h"

namespace GNSS_RTK_ROVER
{
    bool BatteryMonitor::initialized = false;
    bool BatteryMonitor::isCharging = false;
    int8_t BatteryMonitor::percentage;
	float_t BatteryMonitor::voltage;
	int8_t BatteryMonitor::batteryPin;
    std::function<void()> BatteryMonitor::onBatteryFull;
    std::function<void()> BatteryMonitor::onBatteryNotFull;
    std::function<void()> BatteryMonitor::onBatteryZero;
    std::function<void()> BatteryMonitor::onBatteryDead;
	std::function<void(float_t, uint8_t)> BatteryMonitor::onPercentageChanged;

    void BatteryMonitor::start(uint8_t _batteryPin, std::function<void(float_t, uint8_t)> _onPercentageChanged,
        std::function<void()> _onBatteryFull, std::function<void()> _onBatteryNotFull,
        std::function<void()> _onBatteryZero, std::function<void()> _onBatteryDead)
    {
        initialized = true;
        voltage = 0;
        percentage = -1;
        batteryPin = _batteryPin;
        pinMode(batteryPin, INPUT);

        onBatteryFull = _onBatteryFull;
        onBatteryNotFull = _onBatteryNotFull;
        onBatteryZero = _onBatteryZero;
        onPercentageChanged = _onPercentageChanged;
        onBatteryDead = _onBatteryDead;
    }

    void BatteryMonitor::stop()
    {
        initialized = false;
    }

    void BatteryMonitor::checkStatus()
    {
        if(!initialized)  // Skip if not setup yet
            return;

        if(percentage == 0)
            onBatteryZero();

        if(!readAndCalculateVoltage() && percentage != -1)
            return;

        if(voltage < BATTERY_DEAD_VOLT)
            onBatteryDead();

        auto new_percentage = calculatePercentage();
        if(new_percentage != percentage)
        {
            if(new_percentage == 100)
                onBatteryFull();
            else if(percentage == 100)
                onBatteryNotFull();

            percentage = new_percentage;
            onPercentageChanged(voltage, percentage);
        }

        Serial.print("Running Time = "); Serial.print(millis()/60000); Serial.print(" min");Serial.print(" --- ");
        Serial.print("Battery Voltage = "); Serial.print(voltage); Serial.print(" V");
        Serial.print(" --- ("); Serial.print(percentage); Serial.println("%)");
    }

    void BatteryMonitor::setChargingState(bool state)
    {
        isCharging = state;
    }

    bool BatteryMonitor::isBatteryFull()
    {
        return percentage == 100;
    }

    float_t BatteryMonitor::getVoltage()
    {
        return voltage;
    }

    uint8_t BatteryMonitor::getDiscretePercentage()
    {
        return percentage;
    }

    bool BatteryMonitor::readAndCalculateVoltage()
    {
        auto voltageReading = 0;
        for (int i = 0; i < READ_SAMPLES_COUNT; i++)
        {
            voltageReading = voltageReading + analogRead(BATTERYPIN);
        }
        voltageReading = voltageReading / READ_SAMPLES_COUNT;

        float readVoltage = 0;
        if(isCharging)
        {
            readVoltage = ((voltageReading * AREF * VOLTAGE_DIVIDER) / 1023) +  CORRECTION_CHARGING;
        }
            else
        {
            readVoltage = ((voltageReading * AREF * VOLTAGE_DIVIDER) / 1023) +  CORRECTION_NO_CHARGING;
        }

        Serial.print("Running Time = "); Serial.print(millis()/1000); Serial.print(" sec");Serial.print(" --- "); Serial.print("  Voltage = "); Serial.println(readVoltage);

        if(!validateVoltageReading(readVoltage))
        {
            return false;
        }

        voltage = readVoltage;
        return true;
    }

    bool BatteryMonitor::validateVoltageReading(float_t readVoltage)
    {
        auto delta = readVoltage - voltage;
        if(abs(delta) < 0.02)
            return false;
        return true;
    }

	uint8_t BatteryMonitor::calculatePercentage()
    {
        if(isCharging)
        {
            if(voltage >= 4.15)
                return 100;
            else if(voltage >= 3.90)
                return 66;
            else if(voltage >= 3.78)
                return 33;
            else
                return 0;
        }
        else
        {
            if(voltage >= 3.90)
                return 100;
            else if(voltage >= 3.75)
                return 66;
            else if(voltage >= 3.61)
                return 33;
            else
                return 0;
        }
        return -1;
    }
}