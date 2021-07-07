#ifndef _GPS_CONFIG_H_
#define _GPS_CONFIG_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    class GPSConfig
    {
        public:
        static void WakeUp();
        static void Sleep();
        enum SolutionType
        {
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

        static void start(int _serialBaudUart1, int _serialBaudUart2, std::function<void()> onConnected, std::function<void()> onTryingConnection,
            std::function<void(SolutionType)> onSolutionTypeChanged, std::function<void(Mode)> onModeChanged,
            std::function<void(long, long, long)> onCoordsChanged);
        static void stop();
        static SolutionType getSolutionType();
        static Mode getMode();
        static void checkStatus();
        static void configureDefault();

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

        static Mode resolveMode();
        static SolutionType resolveSolutionType();
        static bool resolveCoordinates();

        static SFE_UBLOX_GPS gps;
        static int serialBaudUart1;
        static int serialBaudUart2;

        static bool initialized;
        static SolutionType currSolutionType;
        static Mode currMode;
        static long latitude;
        static long longitude;
        static long height;
        static std::function<void()> onConnected;
        static std::function<void()> onTryingConnection;
        static std::function<void(SolutionType)> onSolutionTypeChanged;
        static std::function<void(Mode)> onModeChanged;
        static std::function<void(long, long, long)> onCoordsChanged;
    };
}

#endif