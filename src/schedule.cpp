#include <vector>
#include "Timer.h"

#include "schedule.h"

namespace GNSS_RTK_ROVER 
{
    void Schedule::AddEvent(unsigned long period, void (*callback)(void))
    {
        auto event = Timer();
        event.every(period, callback);
        this->events.push_back(event);
    }

    void Schedule::Update()
    {
        for(auto it = this->events.begin(); it != this->events.cend(); it++)
        {
            it->update();
        }
    }
}