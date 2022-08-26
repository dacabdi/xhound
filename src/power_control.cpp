#include <functional>

#include "power_control.h"

namespace GNSS_RTK_ROVER
{
    int CPUPowerController::onOffPin;
    int CPUPowerController::mainPowerPin;
    bool CPUPowerController::onOffState;
    int CPUPowerController::chargingStatePin;
    bool CPUPowerController::lastChargingState;
    bool CPUPowerController::justChangedOnOff;
    uint8_t CPUPowerController::epoweroffCount;
    uint64_t CPUPowerController::powerSwitchLastPressed;
    std::function<void(bool)> CPUPowerController::onTurnOnOff;
    std::function<void(bool)> CPUPowerController::onChargingChanged;

    void CPUPowerController::setup(
        int _onOffPin,
        int _mainPowerPin,
        int _chargingStatePin,
        std::function<void(bool)> _onTurnOnOff,
        std::function<void(bool)> _onChargingChanged)
    {
        mainPowerPin = _mainPowerPin;
        pinMode(mainPowerPin, OUTPUT);
        turnOnPowerModule();

        onOffPin = _onOffPin;
        pinMode(onOffPin, INPUT_PULLUP);

        chargingStatePin = _chargingStatePin;
        pinMode(chargingStatePin, INPUT);

        onOffState = false;
        lastChargingState = false;
        onTurnOnOff = _onTurnOnOff;
        epoweroffCount = 0;
        onChargingChanged = _onChargingChanged;
        justChangedOnOff = false;
        attachInterrupt(digitalPinToInterrupt(onOffPin), onOffSwitcher, FALLING);

        if(isCharging())
            turnOff();
        else
            turnOn();
    }

    void CPUPowerController::onOffSwitcher()
    {
        auto now = millis();
        if(now - powerSwitchLastPressed <= 100) // Debouncer
            return;

        if(now - powerSwitchLastPressed <= 1000)
        {
            Serial.println(epoweroffCount + 1);
            epoweroffCount++;
        } else {
            epoweroffCount = 0;
        }

        if(epoweroffCount >= 5)
        {
            Serial.println("Restarting");
            NVIC_SystemReset();
        }

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
        turnOnPowerModule();
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
        auto currChargingState = isCharging();
        if(currChargingState != lastChargingState)
        {
            lastChargingState = currChargingState;
            onChargingChanged(lastChargingState);
        }
    }

    void CPUPowerController::turnOnPowerModule()
    {
        //digitalWrite(mainPowerPin, LOW);
        Serial.println("Setting UP CPU Power Control");
        digitalWrite(mainPowerPin, HIGH);
    }

    void CPUPowerController::turnOffPowerModule()
    {
        //pinMode(mainPowerPin, INPUT); // set pin in High-Z mode
        digitalWrite(mainPowerPin, LOW);
    }

    void PeripheralPowerController::setup(int powerPin, PinStatus defaultState)
    {
        m_powerPin = powerPin;
        pinMode(m_powerPin, OUTPUT);
        digitalWrite(m_powerPin, defaultState);
    }

    void PeripheralPowerController::turnOn()
    {
        digitalWrite(m_powerPin, HIGH);
        Serial.println("Turning Peripherals ON");
        delay(1000);
    }

    void PeripheralPowerController::turnOff()
    {
        digitalWrite(m_powerPin, LOW);
        Serial.println("Turning Peripherals OFF");
        delay(1000);
    }
}
