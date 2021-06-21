#ifndef _SCHEDULE_H_
#define _SCHEDULE_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER 
{
    class ScreenManager
    {
        public:        
        static void setup(std::vector<Component*> _screens);
        static void stop();
        static void mainScreen();
        static void nextScreen();
        static void previousScreen();
        static void refresh();

        private:
        static bool initialized;
        static std::vector<Component*> screens;
        static int8_t currentScreen;
        static int lastMoved;
    };
}

#endif