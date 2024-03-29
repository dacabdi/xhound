#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    class Buzzer
    {
        public:
        Buzzer(int buzzPin) : m_buzzPin(buzzPin) { pinMode(m_buzzPin, OUTPUT); }

        void buzzPowerOn();
        void buzzPowerOff();
        void buzzCharging();
        void buzzNoCharging();
        void buzzBatteryZero();
        void buzzFixedRTK();
        void buzzFloatRTK();
        void buzzBaseMode();
        void buzzRoverMode();
        void buzzBTConnected();
        void buzzBTDisconnected();

        private:
        int m_buzzPin;
    };

    inline void Buzzer::buzzPowerOn()
    {
        tone(m_buzzPin, NOTE_A4, 150); delay(200); tone(m_buzzPin, NOTE_B4, 200); delay(250); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzPowerOff()
    {
        tone(m_buzzPin, NOTE_B4, 150); delay(200); tone(m_buzzPin, NOTE_A4, 200); delay(250); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzCharging()
    {
        tone(m_buzzPin, NOTE_C4, 150); delay(200); tone(m_buzzPin, NOTE_D4, 200); delay(250); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzNoCharging()
    {
        tone(m_buzzPin, NOTE_D4, 150); delay(200); tone(m_buzzPin, NOTE_C4, 200); delay(250); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzBatteryZero()
    {
        tone(m_buzzPin, NOTE_A5, 150); delay(150); tone(m_buzzPin, NOTE_D6, 150); delay(150); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzBTConnected()
    {
        tone(m_buzzPin, NOTE_A6, 150); delay(200); tone(m_buzzPin, NOTE_G6, 200); delay(250); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzBTDisconnected()
    {
        tone(m_buzzPin, NOTE_G6, 150); delay(200); tone(m_buzzPin, NOTE_A6, 200); delay(250); noTone(m_buzzPin);
    }

    inline void Buzzer::buzzBaseMode()
    {
        for(int i = 0; i < 2; i++)
        {
            digitalWrite(m_buzzPin, HIGH);
            delay(20);
            digitalWrite(m_buzzPin, LOW);
            delay(30);
        }
    }

    inline void Buzzer::buzzRoverMode()
    {
        for(int i = 0; i < 4; i++)
        {
            digitalWrite(m_buzzPin, HIGH);
            delay(20);
            digitalWrite(m_buzzPin, LOW);
            delay(30);
        }
    }
}

#endif