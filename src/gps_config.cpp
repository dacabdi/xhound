#include <functional>
#include <utility>

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "gps_config.h"

bool is_base_activated = false;

namespace GNSS_RTK_ROVER
{
    SFE_UBLOX_GNSS GPSConfig::gnss;
    int GPSConfig::serialBaudUart1;
    int GPSConfig::serialBaudUart2;
    bool GPSConfig::initialized = false;
    bool GPSConfig::gnssOff = false;
    GPSConfig::GPSData GPSConfig::data;
    std::function<void()> GPSConfig::onConnected;
    std::function<void()> GPSConfig::onTryingConnection;
    std::function<void(GPSConfig::GPSData&)> GPSConfig::onUpdate;

    void GPSConfig::start(int _serialBaudUart1, int _serialBaudUart2, std::function<void()> _onConnected, std::function<void()> _onTryingConnection,
            std::function<void(GPSConfig::GPSData&)> _onUpdate)
    {
        initialized = true;
        serialBaudUart1 = _serialBaudUart1;
        serialBaudUart2 = _serialBaudUart2;
        onConnected = _onConnected;
        onTryingConnection = _onTryingConnection;
        onUpdate = _onUpdate;
        connect();
    }

    void GPSConfig::stop()
    {
        initialized = false;
    }

    void GPSConfig::configureDefault()
    {
        gnss.factoryDefault();
        configurePorts();
        configureForNMEA();
        gnss.setNavigationFrequency(1);
        configureAntenna();
        gnss.saveConfiguration();
        delay(1000);
        Serial.print("GNSS Protocol version: "); Serial.print(gnss.getProtocolVersionHigh()); Serial.print("."); Serial.println(gnss.getProtocolVersionLow());
        Serial.println("GNSS just got reconfigured to default settings");
    }

    void GPSConfig::Sleep()
    {
        gnss.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX);
        Serial.println("GNSS is SLEEPING ...");
        gnssOff = true;
    }

    void GPSConfig::WakeUp()
    {
        while(gnss.getDateValid() == 0)
        {
            Serial.println("GNSS is waking up ...");
            delay(10);
        }
        Serial.println("GNSS is AWAKE ...");
        Serial.print("Date & Time: ");
        Serial.print(gnss.getYear());
        Serial.print("-");
        Serial.print(gnss.getMonth());
        Serial.print("-");
        Serial.print(gnss.getDay());
        Serial.print(" ");
        Serial.print(gnss.getHour());
        Serial.print(":");
        Serial.print(gnss.getMinute());
        Serial.print(":");
        Serial.print(gnss.getSecond());
        Serial.print(" UTC");
        Serial.println();
        gnssOff = false;
    }

    void GPSConfig::configurePorts()
    {
        gnss.setSerialRate(serialBaudUart1, COM_PORT_UART1);
        gnss.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_RTCM3);
        gnss.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA);

        gnss.setSerialRate(serialBaudUart2, COM_PORT_UART2);
        gnss.setPortInput(COM_PORT_UART2, COM_TYPE_RTCM3);
        gnss.setPortOutput(COM_PORT_UART2, COM_TYPE_RTCM3);

        gnss.setPortInput(COM_PORT_USB, COM_TYPE_UBX);
        gnss.setPortOutput(COM_PORT_USB, COM_TYPE_UBX);

        gnss.setPortInput(COM_PORT_I2C, COM_TYPE_UBX);
        gnss.setPortOutput(COM_PORT_I2C, COM_TYPE_UBX);

        gnss.setPortInput(COM_PORT_SPI, NULL);
        gnss.setPortOutput(COM_PORT_SPI, NULL);
    }

    void GPSConfig::checkStatus()
    {
        if(!initialized) // Skip if not setup yet
            return;

        resolveSolutionType();
        resolveCoordinates();
        resolveSIV();
        resolveDOP();
        resolveReferenceStation();
    
        onUpdate(data);
    }

    GPSConfig::SolutionType GPSConfig::getSolutionType()
    {
        return data.solType;
    }

    GPSConfig::Mode GPSConfig::getMode()
    {
        return data.mode;
    }

    void GPSConfig::resolveCoordinates()
    {
        data.lat   = gnss.getHighResLatitude();
        data.lon   = gnss.getHighResLongitude();
        data.alt   = gnss.getAltitudeMSL() * 0.00328084;
    }

    void GPSConfig::connect()
    {
        while(!gnss.begin())
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
        gnss.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
        Serial.println("Enable UBX_NMEA_GGA");
        // Disable NMEA Messages
        gnss.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_GLL");
        gnss.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_GNS");
        gnss.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_GRS");
        gnss.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_GSA");
        gnss.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_GST");
        gnss.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_GSV");
        gnss.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_ZDA");
        gnss.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_VTG");
        gnss.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
        Serial.println("Disable UBX_NMEA_RMC");
    }

    void GPSConfig::disableUBXNavMsgs()
    {
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_ATT, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_CLOCK, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_EOE, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_GEOFENCE, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_ODO, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_ORB, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_POSECEF, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_RESETODO, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_SIG, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_SVIN, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEBDS, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGAL, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGLO, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEGPS, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMELS, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_VELECEF, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_NAV, UBX_NAV_VELNED, COM_PORT_UART1);
    }

    void GPSConfig::disableUBXRxmMsgs()
    {
        gnss.disableMessage(UBX_CLASS_RXM, UBX_RXM_MEASX, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_RXM, UBX_RXM_PMREQ, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_RXM, UBX_RXM_RLM, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_RXM, UBX_RXM_RTCM, COM_PORT_UART1);
        gnss.disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_UART1);
    }

    void GPSConfig::factoryReset()
    {
        gnss.factoryReset();
        Serial.println("The GNSS was factory reseted, please change code, reload and restart...");
        delay(5000);
        connect();
    }

    void GPSConfig::configureAntenna()
    {
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_VOLTCTRL, 1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_SHORTDET,1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL, 1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_OPENDET,1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_OPENDET_POL, 1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_PWRDOWN,1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL, 1);
        gnss.setVal(UBLOX_CFG_HW_ANT_CFG_RECOVER,1);
    }

    void GPSConfig::configureAsBase()
    {
        gnss.setUART2Output(COM_TYPE_RTCM3);
        gnss.setSerialRate(57600, COM_PORT_UART2);
        gnss.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2 | COM_PORT_USB, 1); //Enable message 1005 to output through UART2, message every second
        gnss.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2 | COM_PORT_USB, 10); //Enable message every 10 seconds
        gnss.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m
        Serial.println("Base Configuration completed");
    }

    void GPSConfig::configureDisableBase()
    {
        gnss.setVal(UBLOX_CFG_TMODE_MODE, 0);  //Turn Off Survey Mode
        gnss.setUART2Output(0);
        gnss.disableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART2 | COM_PORT_USB, 1); //Enable message 1005 to output through UART2, message every second
        gnss.disableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.disableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.disableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.disableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART2 | COM_PORT_USB, 1);
        gnss.disableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART2 | COM_PORT_USB, 10); //Enable message every 10 seconds
        Serial.println("Base Mode Terminated");
    }

    void GPSConfig::resolveSolutionType()
    {
        if(gnssOff)
        {
            data.solType = GnssOff;
            return;
        }
        if(gnss.getDiffSoln())
        {
            auto carrierSolutionType = gnss.getCarrierSolutionType();
            switch(carrierSolutionType)
            {
                case 0:
                    data.solType = DGPS;
                    return;
                case 1:
                    data.solType = FloatRTK;
                    return;
                case 2:
                    data.solType =  FixedRTK;
                    return;
            }
        }
        auto fixType = gnss.getFixType();
        switch(fixType)
        {
            case 1:
                data.solType = DeadReckoning;
                return;
            case 2:
                data.solType = TwoDFix;
                return;
            case 3:
                data.solType = ThreeDFix;
                return;
            case 4:
                data.solType = GNSS;
                return;
            case 5:
                data.solType = TimeFix;
                return;
        }
        data.solType = NoFix;
    }

    void GPSConfig::resolveSIV()
    {
        data.siv = gnss.getSIV();
    }

    void GPSConfig::resolveDOP()
    {
        data.dop = ((float)gnss.getPDOP()/100);
    }

    // lat, lon
    std::pair<String, String> GPSConfig::getLatLonHRPretty()
    {
        int latDegrees = float(data.lat) * std::pow(10, -7);
        int latMinutesTemp = float(data.lat - (latDegrees * std::pow(10, 7))) * 60;
        int latMinutes = latMinutesTemp * std::pow(10, -7);
        float latSeconds = float(latMinutesTemp - (latMinutes * std::pow(10, 7))) * pow(10, -7) * 60;

        String prettyLat = String(latDegrees) + "° " + String(latMinutes) + "\' " + String(latSeconds) + "\""; 

        int lonDegrees = float(data.lon) * std::pow(10, -7);
        int lonMinutesTemp = float(data.lon - (lonDegrees * std::pow(10, 7))) * 60;
        int lonMinutes = lonMinutesTemp * std::pow(10, -7);
        float lonSeconds = float(lonMinutesTemp - (lonMinutes * std::pow(10, 7))) * pow(10, -7) * 60;

        String prettyLon = String(lonDegrees) + "° " + String(lonMinutes) + "\' " + String(lonSeconds) + "\""; 

        return {prettyLat, prettyLon};
    }

    void GPSConfig::resolveReferenceStation()
    {
        if(gnss.getRELPOSNED())
        {
            data.refID = gnss.packetUBXNAVRELPOSNED->data.refStationId;
            data.refDistance = gnss.packetUBXNAVRELPOSNED->data.relPosLength;
        } 
        else
        {
            data.refID = 0;
            data.refDistance = 0;
        }
    }
}
