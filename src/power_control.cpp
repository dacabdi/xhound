#include <functional>

#include "ArduinoLowPower.h"
#include "power_control.h"

namespace GNSS_RTK_ROVER
{
    volatile bool CPUPowerController::m_gotosleep;
    int CPUPowerController::m_onOffPin;
    std::function<void()> CPUPowerController::m_onWake;
    std::function<void()> CPUPowerController::m_onSleep;

    void CPUPowerController::setup(int onOffPin, std::function<void()> onWake, std::function<void()> onSleep)
    {
        m_onOffPin = onOffPin;
        m_gotosleep = true;
        m_onWake = onWake;
        m_onSleep = onSleep;

        pinMode(m_onOffPin, INPUT_PULLUP);
        LowPower.attachInterruptWakeup(digitalPinToInterrupt(m_onOffPin), changeState, FALLING);
    }

    void CPUPowerController::checkForSleep()
    {
        if(m_gotosleep)
        {
            m_onSleep();
            LowPower.sleep();
            m_onWake();
        }
    }

    void CPUPowerController::changeState()
    {
        m_gotosleep = !m_gotosleep;
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
