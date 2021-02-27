#ifndef _SCHEDULE_H_
#define _SCHEDULE_H_

#include "Arduino.h"

#define SCHEDULE_MAX_SLOTS 10

namespace GNSS_RTK_ROVER 
{
    class Schedule 
    {
        public:
        Schedule() : timersCount(0) {}
        void AddEvent(unsigned long period, void (*callback)(void));
        void Update();

        private:
        uint8_t timersCount;
        Timer events[SCHEDULE_MAX_SLOTS];
    };
}

#endif