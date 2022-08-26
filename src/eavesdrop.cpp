#include "eavesdrop.h"

namespace GNSS_RTK_ROVER
{
    // SimpleEavesdropper
    void SimpleEavesdropper::eavesdrop()
    {
        while(m_istream.available())
        {
            // TODO this is POC, we need more deterministic punctuation
            const auto byte = m_istream.read();
            // if(byte == '$')
            // {
            //     Serial.println();
            // }
            // Serial.print((char)byte);
            m_ostream.write(byte);
        }
    }

    // UBXEavesdropper
    void UBXEavesdropper::eavesdrop()
    {
        if (read())
        {
		    write();
            printPacket(Serial);
        }
    }

    void UBXEavesdropper::write()
    {
        m_ostream.write(m_preamble, sizeof(m_preamble));
        m_ostream.write((uint8_t*)&m_header, sizeof(Header));
        m_ostream.write(m_payload, m_header.msg_length);
        m_ostream.write(m_checksum, sizeof(m_checksum));
    }

    bool UBXEavesdropper::read()
    {
        // read a byte from the serial port
        while (m_istream.available())
        {
            m_byte = m_istream.read();
            // identify the packet preamble
            if (m_parserState < sizeof(m_preamble))
            {
                readPreamble();
            }
            else
            {
                // read header
                if ((m_parserState - sizeof(m_preamble)) < sizeof(Header))
                {
                    readHeader();
                }
                else if ((m_parserState - sizeof(m_preamble)) - sizeof(Header) < m_header.msg_length)
                {
                    readPayload();
                }
                else if ((m_parserState - sizeof(m_preamble)) - sizeof(Header) - m_header.msg_length < sizeof(m_checksum))
                {
                    if(readChecksum())
                        return true;
                }
            }
        }
        return false;
    }

    void UBXEavesdropper::printPacket(Stream& ostream)
    {
        ostream.println();
        for(auto i = 0; i < sizeof(m_preamble); i++)
        {
            if(m_preamble[i] <= 0xF)
            	ostream.print(F("0"));
            ostream.print(m_preamble[i], HEX);
            ostream.print(" ");
        }
        ostream.print("\n");

        for(auto i = 0; i < sizeof(Header); i++)
        {
            if(((uint8_t*)&m_header)[i] <= 0xF)
            	ostream.print(F("0"));
            ostream.print(((uint8_t*)&m_header)[i], HEX);
            ostream.print(" ");
        }
        ostream.print("\n");

        for(auto i = 0; i < m_header.msg_length; i++)
        {
            if(m_payload[i] <= 0xF)
            	ostream.print(F("0"));
            ostream.print(m_payload[i], HEX);
            ostream.print(" ");
        }
        ostream.print("\n");

        for(auto i = 0; i < sizeof(m_checksum); i++)
        {
            if(m_checksum[i] <= 0xF)
                ostream.print(F("0"));
            ostream.print(m_checksum[i], HEX);
            ostream.print(" ");
        }
        ostream.print("\n");
    }

    void UBXEavesdropper::calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length)
    {
        CK[0] = 0;
        CK[1] = 0;
        for (uint8_t i = 0; i < length; i++)
        {
            CK[0] += payload[i];
            CK[1] += CK[0];
        }
    }
}