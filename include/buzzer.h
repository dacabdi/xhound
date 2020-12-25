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

        private:
        int m_buzzPin;
    };
    
    inline void Buzzer::buzzPowerOn()
    {
        for(int i = 0; i < 5; i++)
        {
            digitalWrite(m_buzzPin, HIGH); 
            delay(40); 
            digitalWrite(m_buzzPin, LOW); 
            delay(60);
        }
    }

    inline void Buzzer::buzzPowerOff()
    {
        for(int i = 0; i < 3; i++)
        {
            digitalWrite(m_buzzPin, HIGH); 
            delay(100); 
            digitalWrite(m_buzzPin, LOW); 
            delay(50);
        }
    }
}

#endif