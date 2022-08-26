#include <string>

#include "Timer.h"

#include "schedule.h"

namespace GNSS_RTK_ROVER
{
    void Schedule::AddEvent(unsigned long period, void (*callback)(void))
    {
        auto event = Timer();
        event.every(period, callback);
        if(this->timersCount < SCHEDULE_MAX_SLOTS)
        {
            this->events[this->timersCount++] = event;
        }
        else
        {
            Serial.println("Error: schedule ran out of slots");
        }
    }

    void Schedule::Update()
    {
        for(uint8_t timerIndex = 0; timerIndex < this->timersCount; timerIndex++)
        {
            this->Update(timerIndex);
        }
    }

    void Schedule::Update(uint8_t timerIndex)
    {
        this->events[timerIndex].update();
    }
}