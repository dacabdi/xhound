#include <functional>

#include "bluetooth_monitor.h"

namespace GNSS_RTK_ROVER
{
    bool BluetoothMonitor::initialized = false;
	uint8_t BluetoothMonitor::statePin;
    bool BluetoothMonitor::state = false;
	std::function<void()> BluetoothMonitor::onConnected;
    std::function<void()> BluetoothMonitor::onDisconnected;

    void BluetoothMonitor::start(uint8_t _statePin, std::function<void()> _onConnected, std::function<void()> _onDisconnected)
    { 
        initialized = true;
        statePin = _statePin;
        pinMode(statePin, INPUT);

        onConnected = _onConnected;
        onDisconnected = _onDisconnected;
    }

    void BluetoothMonitor::stop()
    { 
        initialized = false;
    }

    void BluetoothMonitor::checkStatus()
    {
        if(!initialized) // Skip if not setup yet
            return;

        auto new_state = digitalRead(statePin);
        if(new_state != state)
        {
            state = new_state;
            if(state)
            {
                onConnected();
            }
            else
            {
                onDisconnected();
            }
        }
    }
}