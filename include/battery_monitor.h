#ifndef _BATTERY_MONITOR_H_
#define _BATTERY_MONITOR_H_

#include "Arduino.h"

#define BATTERYPIN A1
#define AREF 2.23L
#define VOLTAGE_DIVIDER 2.00L
#define CORRECTION_CHARGING -0.06L
#define CORRECTION_NO_CHARGING -0.03L
#define READ_SAMPLES_COUNT 100 //How many reads to get an average
#define BATTERY_DEAD_VOLT 3.60

namespace GNSS_RTK_ROVER
{
	class BatteryMonitor
	{
		public:
		static void start(uint8_t batteryPin, std::function<void(float_t, bool)> onPercentageChanged,
			std::function<void()> onBatteryFull, std::function<void()> onBatteryNotFull,
			std::function<void()> _onBatteryDead);
		static void stop();
		static void checkStatus();
		static void setChargingState(bool state);
		static bool isBatteryFull();
		static float_t getVoltage();

		private:
		static bool readAndCalculateVoltage();
		static bool validateVoltageReading(float_t);

		static bool initialized;
		static bool isCharging;
		static bool batteryFull;
		static float_t voltage;
		static int8_t batteryPin;
		static std::function<void()> onBatteryFull;
		static std::function<void()> onBatteryNotFull;
		static std::function<void()> onBatteryDead;
		static std::function<void(float_t, bool)> onVoltageChanged;
	};

	class BatteryPercentageProvider
	{
		public:
		static uint8_t getBatteryPercentage(float voltage, bool isCharging);
	};
}

#endif