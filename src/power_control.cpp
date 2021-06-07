#include <functional>

#include "power_control.h"

namespace GNSS_RTK_ROVER
{
    int CPUPowerController::onOffPin;
    int CPUPowerController::mainPowerPin;
    bool CPUPowerController::onOffState;
    int CPUPowerController::chargingStatePin;
    bool CPUPowerController::chargingState;
    bool CPUPowerController::justChangedOnOff;
    uint64_t CPUPowerController::powerSwitchLastPressed;
    std::function<void(bool)> CPUPowerController::onTurnOnOff;
    std::function<void(bool)> CPUPowerController::onChargingChanged;

    void CPUPowerController::setup(int _onOffPin, int _mainPowerPin, int _chargingStatePin, std::function<void(bool)> _onTurnOnOff, std::function<void(bool)> _onChargingChanged)
    {
        mainPowerPin = _mainPowerPin;
        pinMode(mainPowerPin, OUTPUT);
        turnOnPowerModule();

        onOffPin = _onOffPin;
        pinMode(onOffPin, INPUT_PULLUP);
        chargingStatePin = _chargingStatePin;
        pinMode(chargingStatePin, INPUT);

        onOffState = false;
        chargingState = false;
        onTurnOnOff = _onTurnOnOff;
        onChargingChanged = _onChargingChanged;
        justChangedOnOff = false;
        attachInterrupt(digitalPinToInterrupt(onOffPin), onOffSwitcher, FALLING);

        if(!isCharging())
            turnOn();
        else
            turnOff();
    }

    void CPUPowerController::onOffSwitcher()
    {
        auto now = millis();
        if(now - powerSwitchLastPressed <= 100) // Debouncer
            return;

        Serial.println("OnOff Switch Pressed");
        if(onOffState)
        {
            if(now - powerSwitchLastPressed <= 1000)
            {
                justChangedOnOff = true;
            }
            powerSwitchLastPressed = now;
        }
        else
        {
            justChangedOnOff = true;
        }
    }

    void CPUPowerController::checkOnOffStatus()
    {
        if(justChangedOnOff)
        {
            if(onOffState)
            {
                turnOff();
            }
            else
            {
                turnOn();
            }
            justChangedOnOff = false;
        }
    }

    void CPUPowerController::turnOn()
    {
        pinMode(mainPowerPin, OUTPUT);
        onOffState = true;
        onTurnOnOff(onOffState);
    }

    void CPUPowerController::turnOff()
    {
        onOffState = false;
        onTurnOnOff(onOffState);
        turnOffPowerModule();
    }

    bool CPUPowerController::isCharging()
    {
        return digitalRead(chargingStatePin);
    }

    void CPUPowerController::checkCharging()
    {
        auto currState = isCharging();
        if(currState != chargingState)
        {
            chargingState = currState;
            onChargingChanged(chargingState);
        }
    }

    void CPUPowerController::turnOnPowerModule()
    {
        digitalWrite(mainPowerPin, LOW);
    }

    void CPUPowerController::turnOffPowerModule()
    {
        pinMode(mainPowerPin, INPUT); // set pin in High-Z mode
    }

    void PeripheralPowerController::setup(int powerPin, PinStatus defaultState)
    {
        m_powerPin = powerPin;
        pinMode(m_powerPin, OUTPUT);
        digitalWrite(m_powerPin, defaultState);
    }

    void PeripheralPowerController::turnOn()
    {
        digitalWrite(m_powerPin, LOW);
    }

    void PeripheralPowerController::turnOff()
    {
        digitalWrite(m_powerPin, HIGH);
    }
}
