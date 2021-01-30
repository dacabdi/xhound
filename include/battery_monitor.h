#ifndef _BATTERY_MONITOR_H_
#define _BATTERY_MONITOR_H_

#include "Arduino.h"

#define BATTERYPIN A1
#define VOLTAGEDIVIDER 2
#define AREF 3.3

namespace GNSS_RTK_ROVER
{
	class BatteryMonitor
	{
		public:
		static void setup(std::function<void(float_t, uint8_t)> onPercentageChanged);
		static void checkStatus();
		static float_t getVoltage();
		static uint8_t getDiscretePercentage();

		private:
		static float_t readAndCalculateVoltage();
		static uint8_t calculatePercentage();
		static void initializeVoltToPercMap();

		static uint8_t percentage;
		static float_t voltage;
		static uint8_t batteryPin;
		static std::map<uint16_t, uint8_t> voltToPercMap;
		static std::function<void(float_t, uint8_t)> onPercentageChanged;
	};
}

#endif