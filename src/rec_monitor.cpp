#include <functional>

#include "rec_monitor.h"

namespace GNSS_RTK_ROVER
{
    bool RecMonitor::initialized = false;
    bool RecMonitor::state = false;
	std::function<void()> RecMonitor::onRecording;
    std::function<void()> RecMonitor::onNotRecording;

    void RecMonitor::start(uint8_t _state, std::function<void()> _onRecording, std::function<void()> _onNotRecording)
    {
        initialized = true;
        state = _state;
        onRecording = _onRecording;
        onNotRecording = _onNotRecording;
    }

    void RecMonitor::stop()
    {
        initialized = false;
    }

    void RecMonitor::checkStatus()
    {
        if(!initialized) // Skip if not setup yet
            return;

        auto new_state = state;
        if(new_state != state)
        {
            state = new_state;
            if(state)
            {
                onRecording();
            }
            else
            {
                onNotRecording();
            }
        }
    }
}