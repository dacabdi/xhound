#ifndef _REC_MONITOR_H_
#define _REC_MONITOR_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
	class RecMonitor
	{
		public:

		static void start(uint8_t _state, std::function<void()> _onRecording, std::function<void()> _onNotRecording);
		static void stop();
		static void checkStatus();

		private:
		static bool initialized;
		static bool state;
		static std::function<void()> onRecording;
		static std::function<void()> onNotRecording;
	};
}

#endif