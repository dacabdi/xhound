#ifndef _GPS_CONFIG_H_
#define _GPS_CONFIG_H_

#include "Arduino.h"

#define GNSS_CALLS_MAXWAIT 250

namespace GNSS_RTK_ROVER
{
    class GPSConfig
    {
        public:
        enum SolutionType
        {
            GnssOff,
            UnknownSolutionType,
            NoFix,
            DeadReckoning,
            TwoDFix,
            ThreeDFix,
            GNSS,
            DGPS,
            TimeFix,
            FloatRTK,
            FixedRTK
        };

        enum Mode
        {
            UnknownMode,
            Rover,
            Base
        };

        struct GPSData
        {
            SolutionType solType;
            Mode mode;
            long lat;
            long lon;
            float_t alt;
            uint16_t siv;
            float_t hdop;
            float_t vdop;
            float_t pdop;
            uint16_t refID;
            int32_t refDistance;
        };

        static void start(int _serialBaudUart1, int _serialBaudUart2, std::function<void()> onConnected, std::function<void()> onTryingConnection,
            std::function<void(GPSData&)> onUpdate);
        static void stop();
        static void checkStatus();
        static void checkUblox();
        static void checkUbloxCallbacks();
        static void configureDefault();
        static void WakeUp();
        static void Sleep();

        static SolutionType getSolutionType();
        static Mode getMode();
        static std::pair<String, String> getLatLonHRPretty();

        private:
        static void connect();
        static void factoryReset();
        static void configurePorts();
        static void configureAntenna();
        static void configureForNMEA();
        static void configureNMEAMsgs();
        static void disableUBXNavMsgs();
        static void disableUBXRxmMsgs();
        static void configureAsBase();
        static void configureDisableBase();

        static void setAutoCalls();
        static void resolvePVT(UBX_NAV_PVT_data_t packet);
        static void resolveDOP(UBX_NAV_DOP_data_t packet);
        static void resolveRELPOSNED(UBX_NAV_RELPOSNED_data_t packet);
        static void resolveSolutionType();
        static void resolveCoordinates();
        static void resolveSIV();
        static void resolveDOP();
        static void resolveReferenceStation();

        static SFE_UBLOX_GNSS gnss;
        static int serialBaudUart1;
        static int serialBaudUart2;

        static bool initialized;
        static bool gnssOff;
        static GPSData data;
        static std::function<void()> onConnected;
        static std::function<void()> onTryingConnection;
        static std::function<void(GPSData&)> onUpdate;
    };
}

#endif