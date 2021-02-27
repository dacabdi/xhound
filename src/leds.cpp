#include "leds.h"

namespace GNSS_RTK_ROVER
{
    int LED::instancesCount = 0;
    LED* LED::instances[MAX_LED_COUNT];

    void LED::addInstance(LED* led)
    {
        if(instancesCount < MAX_LED_COUNT)
        {
            instances[instancesCount++] = led;
        }
        else
        {
            Serial.println("Max LED count reached");
        }
    }

    void LED::refreshInstances()
    {
        for(int i = 0; i < instancesCount; i++)
        {
            instances[i]->refresh();
        }
    }

    void LED::set(uint8_t _intOn, uint16_t _blinkPeriod, uint8_t _intOff)
    {
        this->on = true;
        this->intOn = _intOn;
        this->intOff = _intOff;
        this->blinkPeriod = _blinkPeriod;
        this->lastBlink = millis();
        analogWrite(this->ledPin, this->intOn);
    }

    void LED::refresh()
    {
        if(this->blinkPeriod)
        {
            auto now = millis();
            if(now - this->lastBlink >= this->blinkPeriod)
            {
                this->lastBlink = now;
                this->on = !this->on;
                analogWrite(this->ledPin, on ? this->intOn : this->intOff);
            }
        }
    }
}