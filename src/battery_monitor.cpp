#include <functional>
#include <map>

#include "battery_monitor.h"

namespace GNSS_RTK_ROVER
{
    bool BatteryMonitor::initialized = false;
    bool BatteryMonitor::isCharging = false;
    bool BatteryMonitor::batteryFull = false;
	float_t BatteryMonitor::voltage;
	int8_t BatteryMonitor::batteryPin;
    std::function<void()> BatteryMonitor::onBatteryFull;
    std::function<void()> BatteryMonitor::onBatteryNotFull;
    std::function<void()> BatteryMonitor::onBatteryDead;
	std::function<void(float_t, bool)> BatteryMonitor::onVoltageChanged;

    void BatteryMonitor::start(uint8_t _batteryPin, std::function<void(float_t, bool)> _onVoltageChanged,
        std::function<void()> _onBatteryFull, std::function<void()> _onBatteryNotFull,
        std::function<void()> _onBatteryDead)
    {
        initialized = true;
        batteryFull = false;
        voltage = 0;
        batteryPin = _batteryPin;
        pinMode(batteryPin, INPUT);

        onBatteryFull = _onBatteryFull;
        onBatteryNotFull = _onBatteryNotFull;
        onVoltageChanged = _onVoltageChanged;
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

        if(!readAndCalculateVoltage() && voltage != 0)
            return;

        if(voltage < BATTERY_DEAD_VOLT)
            onBatteryDead();

        if(!batteryFull || !isCharging || voltage < 4.00)
        {
            onVoltageChanged(voltage, isCharging);
        }
        if(voltage >= 4.15 && !batteryFull)
        {
            batteryFull = true;
            onBatteryFull();
        }
        if(voltage < 4.15 && batteryFull && (!isCharging || voltage < 4.00))
        {
            batteryFull = false;
            onBatteryNotFull();
        }

        Serial.print("Valid:  Running Time = "); Serial.print(millis()/60000); Serial.print(" min");Serial.print(" --- ");
        Serial.print("Battery Voltage = "); Serial.print(voltage); Serial.println(" V");
    }

    void BatteryMonitor::setChargingState(bool state)
    {
        if(!state)
        {
            batteryFull = false;
        }
        isCharging = state;
        onVoltageChanged(voltage, isCharging);
    }

    bool BatteryMonitor::isBatteryFull()
    {
        return batteryFull;
    }

    float_t BatteryMonitor::getVoltage()
    {
        return voltage;
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

        Serial.print("Running Time = "); Serial.print(millis()/1000); Serial.print(" sec");Serial.print(" --- ");
        Serial.print("Battery Voltage = "); Serial.print(readVoltage); Serial.println(" V");

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

    uint8_t BatteryPercentageProvider::getBatteryPercentage(float voltage, bool isCharging)
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