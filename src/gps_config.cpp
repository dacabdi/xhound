#include <functional>

#include "SparkFun_Ublox_Arduino_Library.h"
#include "gps_config.h"

bool is_base_activated = false;

namespace GNSS_RTK_ROVER 
{
    GPSConfig::GPSConfig(int serialBaud, std::function<void()> onConnected, std::function<void()> onTryingConnection,
        std::function<void()> onReset, std::function<void()> onNMEA, std::function<void()> onUBX)
        : m_serialBaud(serialBaud), m_onConnected(onConnected), m_onTryingConnection(onTryingConnection), m_onReset(onReset), 
        m_onNMEA(onNMEA), m_onUBX(onUBX), m_usingUBX(false), m_checkCount(0)
    {
        initialize();
    }

    void GPSConfig::initialize()
    {
        connect();
        m_gps.setSerialRate(m_serialBaud, COM_PORT_UART1);
        m_gps.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);
        m_gps.setNavigationFrequency(5);

        configureDisableBase();
        configureUnusedPorts();
        configureI2C();
        configureForNMEA();
        configureAntenna();
        m_gps.saveConfiguration();

        //m_gps.getEsfInfo
    }

    void GPSConfig::checkForStatus()
    {
        if(!m_checkCount)
        {
            if(m_usingUBX && m_gps.getVal8(0x10740001, 1) == 0)
            {
                m_usingUBX = !m_usingUBX;
                configureForNMEA();
            }
            else if(!m_usingUBX && m_gps.getVal8(0x10740001, 1) == 1)
            {
                m_usingUBX = !m_usingUBX;
                configureForUBX();
            }
        }
        m_checkCount = (m_checkCount + 1) % 200;
    }

    void GPSConfig::connect()
    {
        while(!m_gps.begin())
        {
            m_onTryingConnection();
            delay(100);
        }
        m_onConnected();
    }

    void GPSConfig::configureUnusedPorts()
    {
        m_gps.setSPIOutput(0);
        m_gps.setUART2Output(0);
        m_gps.setUSBOutput(COM_TYPE_NMEA | COM_TYPE_UBX | COM_TYPE_RTCM3);
    }

    void GPSConfig::configureI2C()
    {
        m_gps.setI2COutput(COM_TYPE_UBX);    
    }
    
    void GPSConfig::configureForNMEA()
    {
        disableUBXNavMsgs();
        disableUBXRxmMsgs();
        configureNMEAMsgs();
        m_onNMEA();
    }

    void GPSConfig::configureForUBX()
    {
        m_onUBX();
    }

    void GPSConfig::configureNMEAMsgs()
    {
        // Enable NMEA Messages
        m_gps.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
        // Disable NMEA Messages
        m_gps.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
        m_gps.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXNavMsgs()
    {
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_ATT, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_CLOCK, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_EOE, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_GEOFENCE, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_ODO, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_ORB, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_POSECEF, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_RESETODO, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_SIG, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_SVIN, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEBDS, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGAL, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGLO, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGPS, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMELS, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_VELECEF, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_NAV, UBX_NAV_VELNED, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXRxmMsgs()
    {
        m_gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_MEASX, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_PMREQ, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_RLM, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_RTCM, COM_PORT_UART1);
        m_gps.disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_UART1);       
    }

    void GPSConfig::factoryReset()
    {
        m_gps.factoryReset();
        delay(5000);
        m_onReset();
        connect();
    }

    void GPSConfig::configureAntenna()
    {
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_VOLTCTRL, 1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_SHORTDET,1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL, 1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_OPENDET,1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_OPENDET_POL, 1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_PWRDOWN,1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL, 1);
        m_gps.setVal(UBLOX_CFG_HW_ANT_CFG_RECOVER,1);
    }

    void GPSConfig::configureAsBase()
    {
        m_gps.setUART2Output(COM_TYPE_RTCM3);
        m_gps.setSerialRate(57600, COM_PORT_UART2);
        m_gps.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2 | COM_PORT_USB, 1); //Enable message 1005 to output through UART2, message every second
        m_gps.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2 | COM_PORT_USB, 10); //Enable message every 10 seconds
        m_gps.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m
        Serial.println("Base Configuration completed");
    }

    void GPSConfig::configureDisableBase()
    {
        m_gps.setVal(UBLOX_CFG_TMODE_MODE, 0);  //Turn Off Survey Mode
        m_gps.setUART2Output(0);
        m_gps.disableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2 | COM_PORT_USB, 1); //Enable message 1005 to output through UART2, message every second
        m_gps.disableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.disableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.disableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.disableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2 | COM_PORT_USB, 1);
        m_gps.disableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2 | COM_PORT_USB, 10); //Enable message every 10 seconds
        Serial.println("Base Mode Terminated");
    }

    uint8_t GPSConfig::getSolution()
    {
        return m_gps.getCarrierSolutionType();
    }

    bool GPSConfig::check_isBaseActivated()
    {
        is_base_activated = m_gps.svin.active;
        return is_base_activated;
    }

    float GPSConfig::meanAccuracy()
    {
        m_gps.getSurveyStatus(UBX_NAV_SVIN);
        bool is_base_activated = m_gps.svin.active;
        if(is_base_activated)
        {
            Serial.print("Base Status = Acitvated");
            Serial.print(" --- Time Ellapsed = ");
            Serial.print(m_gps.svin.observationTime);
            Serial.print(" --- Mean 3D Accuracy = "); Serial.println(m_gps.svin.meanAccuracy);
            if(m_gps.svin.valid)
            { 
                Serial.println(" --- Position = VALID");
            }
            else
            {
               Serial.println(" --- Position = NO VALID"); 
            }
            return m_gps.svin.meanAccuracy;
        }
        else
        {
            return 0;
        }
    } 
}