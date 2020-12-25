#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    class CPUPowerController
    {
        public:
        static void setup(int onOffPin, std::function<void()> onWake, std::function<void()> onSleep);
        static void checkForSleep();

        private:
        CPUPowerController();
        static void changeState();

        static volatile bool m_gotosleep;
        static int m_onOffPin;

        static std::function<void()> m_onWake;
        static std::function<void()> m_onSleep;
    };

    class PeriferalPowerController
    {
      public:
      PeriferalPowerController(int powerPin) : m_powerPin(powerPin) { pinMode(m_powerPin, OUTPUT); }

      void turnOn();
      void turnOff();

      private:
      int m_powerPin;
    };
}

#endif