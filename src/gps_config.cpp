#include <functional>

#include "SparkFun_Ublox_Arduino_Library.h"
#include "gps_config.h"

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
        configureUnusedPorts();
        configureI2C();
        configureNMEAMsgs();
        m_gps.saveConfiguration();
        
        configureForNMEA();
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
        m_gps.setUART2Output(0);              
    }

    void GPSConfig::configureI2C()
    {
        m_gps.setI2COutput(COM_TYPE_UBX);    
    }
    
    void GPSConfig::configureForNMEA()
    {
        m_onNMEA();
        m_gps.setUART1Output(COM_TYPE_NMEA);                
    }

    void GPSConfig::configureForUBX()
    {
        m_onUBX();
        m_gps.setUART1Output(COM_TYPE_UBX);                
    }

    // TODO handle as composite
    void GPSConfig::configureForUBXAndNMEA()
    {
        m_gps.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);                
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

    void GPSConfig::factoryReset()
    {
        m_gps.factoryReset();
        delay(5000);
        m_onReset();
        connect();
    }
}