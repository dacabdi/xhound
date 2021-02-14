#include <functional>

#include "ArduinoLowPower.h"
#include "power_control.h"

namespace GNSS_RTK_ROVER
{
    int CPUPowerController::onOffPin;
    bool CPUPowerController::onOffState;
    int CPUPowerController::chargingStatePin;
    bool CPUPowerController::chargingState;
    uint64_t CPUPowerController::powerSwitchLastPressed;
    std::function<void(bool)> CPUPowerController::onTurnOnOff;
    std::function<void(bool)> CPUPowerController::onChargingChanged;

    void CPUPowerController::setup(int _onOffPin, int _chargingStatePin, std::function<void(bool)> _onTurnOnOff, std::function<void(bool)> _onChargingChanged)
    {
        onOffState = false;
        onOffPin = _onOffPin;
        chargingStatePin = _chargingStatePin;
        onTurnOnOff = _onTurnOnOff;
        onChargingChanged = _onChargingChanged;
        attachInterrupt(digitalPinToInterrupt(onOffPin), onOffSwitcher, RISING);

        if(!isCharging())
        {
            turnOn();
        }
        else
        {
            onChargingChanged(chargingState);
        }

        Serial.print("WokeUp, device onoffStatus: ");
        Serial.println(onOffState);
    }

    void CPUPowerController::onOffSwitcher()
    {
        Serial.println("OnOff Switch");
        auto now = millis();
        if(now - powerSwitchLastPressed <= 50) // Debouncer
            return;
        
        if(onOffState)
        {
            if(now - powerSwitchLastPressed <= 1000)
            {
                turnOff();
            }
            powerSwitchLastPressed = now;
        }
        else
        {
            turnOn();
        }
    }

    void CPUPowerController::turnOn()
    {
        Serial.println("Turn On");
        onOffState = true;
        onTurnOnOff(onOffState);
    }

    void CPUPowerController::turnOff()
    {
        Serial.println("Turn Off");
        onOffState = false;
        onTurnOnOff(onOffState);
    }

    bool CPUPowerController::isCharging()
    {
        return digitalRead(chargingStatePin);
    }

    void CPUPowerController::checkCharging()
    {
        Serial.print("Checking...  ");
        auto currState = isCharging();
        Serial.println(currState);
        if(currState != chargingState)
        {
            chargingState = currState;
            onChargingChanged(chargingState);
        }    
    }

    void PeriferalPowerController::turnOn()
    {
        digitalWrite(m_powerPin, LOW);
    }

    void PeriferalPowerController::turnOff()
    {
        digitalWrite(m_powerPin, HIGH);
    }
}
