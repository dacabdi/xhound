#ifndef _BATTERY_MONITOR_H_
#define _BATTERY_MONITOR_H_

#include "Arduino.h"

#define BATTERYPIN A1
#define VOLTAGEDIVIDER 1.308
#define AREF 3.28
#define READ_SAMPLES_COUNT 30

namespace GNSS_RTK_ROVER
{
	class BatteryMonitor
	{
		public:
		static void start(uint8_t batteryPin, std::function<void(float_t, uint8_t)> onPercentageChanged, 
			std::function<void()> onBatteryFull, std::function<void()> onBatteryNotFull, std::function<void()> onBatteryZero);
		static void stop();
		static void checkStatus();
		static void setChargingState(bool state);
		static bool isBatteryFull();
		static float_t getVoltage();
		static uint8_t getDiscretePercentage();

		private:
		static bool readAndCalculateVoltage();
		static bool validateVoltageReading(float_t);
		static uint8_t calculatePercentage();
		static void initializeVoltToPercMap();

		static bool initialized;
		static bool isCharging;
		static int8_t percentage;
		static float_t voltage;
		static int8_t batteryPin;
		static std::function<void()> onBatteryFull;
		static std::function<void()> onBatteryNotFull;
		static std::function<void()> onBatteryZero;
		static std::function<void(float_t, uint8_t)> onPercentageChanged;
	};
}

#endif