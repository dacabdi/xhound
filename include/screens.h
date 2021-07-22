#ifndef _SCREENS_H_
#define _SCREENS_H_

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
        static void goToScreen(uint8_t screenNumber);

        static bool initialized;
        static std::vector<Component*> screens;
        static int8_t currentScreen;
        static int8_t prevScreen;
        static int lastMoved;
    };
}

#endif