#ifndef _BLUETOOTH_MONITOR_H_
#define _BLUETOOTH_MONITOR_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
	class BluetoothMonitor
	{
        public:
        static void setup(uint8_t _statePin, std::function<void()> _onConnected, std::function<void()> _onDisconnected);
        static void checkStatus();

        private:
        static bool initialized;
        static bool state;
        static uint8_t statePin;
        static std::function<void()> onConnected;
        static std::function<void()> onDisconnected;
	};
}

#endif