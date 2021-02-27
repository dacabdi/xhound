#include <functional>

#include "SparkFun_Ublox_Arduino_Library.h"
#include "gps_config.h"

bool is_base_activated = false;

namespace GNSS_RTK_ROVER 
{
    SFE_UBLOX_GPS GPSConfig::gps;
    int GPSConfig::serialBaudUart1;
    int GPSConfig::serialBaudUart2;
    bool GPSConfig::initialized = false;
    GPSConfig::SolutionType GPSConfig::currSolutionType = GPSConfig::UNKNOWN;
    GPSConfig::Mode GPSConfig::currMode = GPSConfig::ROVER;
    std::function<void()> GPSConfig::onConnected;
    std::function<void()> GPSConfig::onTryingConnection;
    std::function<void(GPSConfig::SolutionType)> GPSConfig::onSolutionTypeChanged;
    std::function<void(GPSConfig::Mode)> GPSConfig::onModeChanged;

    void GPSConfig::setup(int _serialBaudUart1, int _serialBaudUart2, std::function<void()> _onConnected, std::function<void()> _onTryingConnection,
            std::function<void(GPSConfig::SolutionType)> _onSolutionTypeChanged, std::function<void(GPSConfig::Mode)> _onModeChanged)
    {
        initialized = true;
        serialBaudUart1 = _serialBaudUart1;
        serialBaudUart2 = _serialBaudUart2;
        onConnected = _onConnected;
        onTryingConnection = _onTryingConnection;
        onSolutionTypeChanged = _onSolutionTypeChanged;
        onModeChanged = _onModeChanged;
        connect();
    }

    void GPSConfig::configureDefault()
    {
        gps.factoryDefault();
        configurePorts();
        gps.setNavigationFrequency(5);
        configureAntenna();
        gps.saveConfiguration();
    }

    void GPSConfig::configurePorts()
    {
        gps.setSerialRate(serialBaudUart1, COM_PORT_UART1);
        gps.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_RTCM3);
        gps.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA);

        gps.setSerialRate(serialBaudUart2, COM_PORT_UART2);
        gps.setPortInput(COM_PORT_UART2, COM_TYPE_RTCM3);
        gps.setPortOutput(COM_PORT_UART2, COM_TYPE_RTCM3);

        gps.setPortInput(COM_PORT_USB, COM_TYPE_UBX);
        gps.setPortOutput(COM_PORT_USB, COM_TYPE_UBX);

        gps.setPortInput(COM_PORT_I2C, COM_TYPE_UBX);
        gps.setPortOutput(COM_PORT_I2C, COM_TYPE_UBX);

        gps.setPortInput(COM_PORT_SPI, NULL);
        gps.setPortOutput(COM_PORT_SPI, NULL);
    }

    void GPSConfig::checkStatus()
    {
        if(!initialized) // Skip if not setup yet
            return;

        auto solType = getSolutionType();
        Serial.print("Solution Type: ");
        Serial.println(solType);
        if(solType != currSolutionType)
        {
            currSolutionType = solType;
            onSolutionTypeChanged(currSolutionType);
        }
        auto mode = getMode();
        Serial.print("Mode: ");
        Serial.println(mode);
        if(mode != currMode)
        {
            currMode = mode;
            onModeChanged(currMode);
        }
    }

    void GPSConfig::connect()
    {
        while(!gps.begin())
        {
            onTryingConnection();
            delay(100);
        }
        onConnected();
    }
    
    void GPSConfig::configureForNMEA()
    {
        disableUBXNavMsgs();
        disableUBXRxmMsgs();
        configureNMEAMsgs();
    }

    void GPSConfig::configureNMEAMsgs()
    {
        // Enable NMEA Messages
        gps.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
        // Disable NMEA Messages
        gps.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
        gps.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXNavMsgs()
    {
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_ATT, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_CLOCK, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_EOE, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_GEOFENCE, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_ODO, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_ORB, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_POSECEF, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_RESETODO, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_SIG, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_SVIN, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEBDS, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGAL, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGLO, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGPS, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMELS, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_VELECEF, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_VELNED, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXRxmMsgs()
    {
        gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_MEASX, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_PMREQ, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_RLM, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_RTCM, COM_PORT_UART1);
        gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_UART1);       
    }

    void GPSConfig::factoryReset()
    {
        gps.factoryReset();
        delay(5000);
        connect();
    }

    void GPSConfig::configureAntenna()
    {
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_VOLTCTRL, 1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_SHORTDET,1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL, 1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_OPENDET,1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_OPENDET_POL, 1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_PWRDOWN,1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL, 1);
        gps.setVal(UBLOX_CFG_HW_ANT_CFG_RECOVER,1);
    }

    void GPSConfig::configureAsBase()
    {
        gps.setUART2Output(COM_TYPE_RTCM3);
        gps.setSerialRate(57600, COM_PORT_UART2);
        gps.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2 | COM_PORT_USB, 1); //Enable message 1005 to output through UART2, message every second
        gps.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2 | COM_PORT_USB, 10); //Enable message every 10 seconds
        gps.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m
        Serial.println("Base Configuration completed");
    }

    void GPSConfig::configureDisableBase()
    {
        gps.setVal(UBLOX_CFG_TMODE_MODE, 0);  //Turn Off Survey Mode
        gps.setUART2Output(0);
        gps.disableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2 | COM_PORT_USB, 1); //Enable message 1005 to output through UART2, message every second
        gps.disableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.disableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.disableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.disableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2 | COM_PORT_USB, 1);
        gps.disableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2 | COM_PORT_USB, 10); //Enable message every 10 seconds
        Serial.println("Base Mode Terminated");
    }

    GPSConfig::SolutionType GPSConfig::getSolutionType()
    {
        return (SolutionType)gps.getCarrierSolutionType();
    }

    GPSConfig::Mode GPSConfig::getMode()
    {
        return (Mode)gps.svin.active;
    }
}