#ifndef _LEDS_H_
#define _LEDS_H_

#include "Arduino.h"

#define MAX_LED_COUNT 6

namespace GNSS_RTK_ROVER
{
	class LED
	{
		public:
		LED() {};
		explicit LED(int _ledPin) : ledPin(_ledPin), intOn(0) {pinMode(ledPin, OUTPUT); addInstance(this); }
		void set(uint8_t intOn, uint16_t blinkPeriod = 0, uint8_t intOff = 0);
		void refresh();

		static void refreshInstances();

		private:
		int ledPin;
		bool on;
		uint8_t intOn;
		uint8_t intOff;
		uint16_t blinkPeriod;
		uint32_t lastBlink;

		static void addInstance(LED* led);
		static LED* instances[MAX_LED_COUNT];
		static int instancesCount;
	};
}

#endif