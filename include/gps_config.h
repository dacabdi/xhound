#ifndef _GPS_CONFIG_H_
#define _GPS_CONFIG_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    class GPSConfig
    {
        public:
        GPSConfig(int serialBaud, std::function<void()> onConnected, std::function<void()> onTryingConnection,
            std::function<void()> onReset, std::function<void()> onNMEA, std::function<void()> onUBX);
        void initialize();
        void factoryReset();
        void checkForStatus();
        void configureForNMEA();
        void configureAsBase();
        void configureDisableBase();
        uint8_t getSolution();
        float meanAccuracy();
        bool check_isBaseActivated();

        private:
        void connect();
        void configureUnusedPorts();
        void configureI2C();
        void configureNMEAMsgs();
        void disableUBXNavMsgs();
        void disableUBXRxmMsgs();
        void configureForUBX();
        void configureAntenna();
                
        SFE_UBLOX_GPS m_gps;
        int m_serialBaud;
        bool m_usingUBX;
        int m_checkCount;
        std::function<void()> m_onConnected;
        std::function<void()> m_onTryingConnection;
        std::function<void()> m_onReset;
        std::function<void()> m_onNMEA;
        std::function<void()> m_onUBX;
    };
}

#endif