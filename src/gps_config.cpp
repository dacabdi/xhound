#include <functional>

#include "SparkFun_Ublox_Arduino_Library.h"
#include "gps_config.h"

namespace GNSS_RTK_ROVER 
{
    GPSConfig::GPSConfig(SFE_UBLOX_GPS* gps, int serialBaud, std::function<void()> onConnected, std::function<void()> onTryingConnection,
        std::function<void()> onReset, std::function<void()> onNMEA, std::function<void()> onUBX)
        : m_gps(gps), m_serialBaud(serialBaud), m_onConnected(onConnected), m_onTryingConnection(onTryingConnection), m_onReset(onReset), 
        m_onNMEA(onNMEA), m_onUBX(onUBX), m_usingUBX(false), m_checkCount(0)
    {
        initialize();
    }

    void GPSConfig::initialize()
    {
        connect();
        m_gps->setSerialRate(m_serialBaud, COM_PORT_UART1);
        m_gps->setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);                
        configureUnusedPorts();
        configureI2C();
        m_gps->saveConfiguration();
        
        configureForNMEA();
    }

    void GPSConfig::checkForStatus()
    {
        if(!m_checkCount)
        {
            if(m_usingUBX && m_gps->getVal8(0x10740001, 1) == 0)
            {
                m_usingUBX = !m_usingUBX;
                configureForNMEA();
            }
            else if(!m_usingUBX && m_gps->getVal8(0x10740001, 1) == 1)
            {
                m_usingUBX = !m_usingUBX;
                configureForUBX();
            }
        }
        m_checkCount = (m_checkCount + 1) % 200;
    }

    void GPSConfig::connect()
    {
        while(!m_gps->begin())
        {
            m_onTryingConnection();
            delay(100);
        }
        m_onConnected();
    }

    void GPSConfig::configureUnusedPorts()
    {
        m_gps->setUART2Output(0);
        m_gps->setSPIOutput(0);
    }

    void GPSConfig::configureI2C()
    {
        m_gps->setI2COutput(COM_TYPE_UBX);    
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
        m_gps->enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
        // Disable NMEA Messages
        m_gps->disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
        m_gps->disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXNavMsgs()
    {
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_ATT, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_CLOCK, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_EOE, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_GEOFENCE, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_ODO, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_ORB, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_POSECEF, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_RESETODO, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_SIG, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_SVIN, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEBDS, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGAL, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGLO, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGPS, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMELS, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_VELECEF, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_NAV, UBX_NAV_VELNED, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXRxmMsgs()
    {
        m_gps->disableMessage(UBX_CLASS_RXM, UBX_RXM_MEASX, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_RXM, UBX_RXM_PMREQ, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_RXM, UBX_RXM_RLM, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_RXM, UBX_RXM_RTCM, COM_PORT_UART1);
        m_gps->disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_UART1);       
    }

    void GPSConfig::factoryReset()
    {
        m_gps->factoryReset();
        delay(5000);
        m_onReset();
        connect();
    }

    uint8_t GPSConfig::getSolution()
    {
        return m_gps->getCarrierSolutionType();
    }
}