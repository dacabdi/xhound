#ifndef _BATTERY_MONITOR_H_
#define _BATTERY_MONITOR_H_

#include "Arduino.h"

#define BATTERYPIN A1
#define VOLTAGEDIVIDER 1.3
#define AREF 3.26

namespace GNSS_RTK_ROVER
{
	class BatteryMonitor
	{
		public:
		static void start(uint8_t batteryPin, std::function<void(float_t, uint8_t)> onPercentageChanged);
		static void stop();
		static void checkStatus();
		static float_t getVoltage();
		static uint8_t getDiscretePercentage();

		private:
		static float_t readAndCalculateVoltage();
		static uint8_t calculatePercentage();
		static void initializeVoltToPercMap();

		static bool initialized;
		static int8_t percentage;
		static float_t voltage;
		static int8_t batteryPin;
		static std::map<uint16_t, uint8_t> voltToPercMap;
		static std::function<void(float_t, uint8_t)> onPercentageChanged;
	};
}

#endif