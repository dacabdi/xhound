#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    class CPUPowerController
    {
        public:
        static void setup(int onOffPin, int mainPowerPin, int chargingStatePin, std::function<void(bool)> _onTurnOnOff, std::function<void(bool)> _onChargingChanged);
        static bool isCharging();
        static void checkOnOffStatus();
        static void checkCharging();
        static void turnOn();
        static void turnOff();

        private:
        CPUPowerController();
        static void turnOnPowerModule();
        static void turnOffPowerModule();
        static void onOffSwitcher();

        static int onOffPin;
        static int mainPowerPin;
        static int chargingStatePin;
        static bool onOffState;
        static bool chargingState;

        static uint64_t powerSwitchLastPressed;
        static uint8_t epoweroffCount;
        static bool justChangedOnOff;
        static std::function<void(bool)> onTurnOnOff;
        static std::function<void(bool)> onChargingChanged;
    };

    class PeripheralPowerController
    {
        public:
        void setup(int powerPin, PinStatus defaultState);
        void turnOn();
        void turnOff();

        private:
        int m_powerPin;
    };
}

#endif
