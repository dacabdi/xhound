#ifndef _SCHEDULE_H_
#define _SCHEDULE_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER 
{
    class Schedule 
    {
        public:
        void AddEvent(unsigned long period, void (*callback)(void));
        void Update();

        private:
        std::vector<Timer> events;
    };
}

#endif