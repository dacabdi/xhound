#ifndef _EAVESDROPPER_H_
#define _EAVESDROPPER_H_
  
#include "Arduino.h"

namespace GNSS_RTK_ROVER 
{
    class Eavesdropper
    {
        public:
        virtual void eavesdrop() = 0;
    };

    class SimpleEavesdropper : public Eavesdropper
    {
        public:
        SimpleEavesdropper(Stream &istream, Stream& ostream) :  m_istream(istream), m_ostream(ostream) {}
        SimpleEavesdropper(Stream &iostream) : m_istream(iostream), m_ostream(iostream) {}

        virtual void eavesdrop() override;

        protected:
        Stream& m_istream;
        Stream& m_ostream;
    };

    class UBXEavesdropper : public SimpleEavesdropper 
    {
        public:
        UBXEavesdropper(Stream& istream, Stream& ostream) : SimpleEavesdropper(istream, ostream) { resetParserAndTmpCK(); }
        UBXEavesdropper(Stream& iostream) : SimpleEavesdropper(iostream) { resetParserAndTmpCK(); }

        virtual void eavesdrop() override;
        void printPacket(Stream& ostream);

        private:
        void write();
        bool read();
        void calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length);
        void resetParserAndTmpCK();
        void addToTempChecksum();

        void readPreamble();
        void readHeader();
        void readPayload();        
        bool readChecksum();

        uint16_t headerIndex();
        uint16_t payloadIndex();
        uint16_t checksumIndex();

        struct Header {
            uint8_t msg_class;
            uint8_t msg_id;
            uint16_t msg_length;
        };

        uint16_t m_parserState;
        uint8_t m_byte;

        const uint8_t m_preamble[2] = {0xB5, 0x62};
        Header m_header;
        uint8_t m_payload[512];
        uint8_t m_checksum[2];
        uint8_t m_tempChecksum[2];

    };

    inline void UBXEavesdropper::resetParserAndTmpCK() 
    {
        m_parserState = 0;
        m_tempChecksum[0] = 0;
        m_tempChecksum[1] = 0;
    }

    inline void UBXEavesdropper::addToTempChecksum() 
    {
        m_tempChecksum[0] += m_byte;
        m_tempChecksum[1] += m_tempChecksum[0];
    }

    inline void UBXEavesdropper::readPreamble()
    {
        if(m_byte == m_preamble[m_parserState])
        {
            m_parserState++;
        }
        else
        {
            m_parserState = 0;
        }
    }

    inline void UBXEavesdropper::readHeader()
    {
        ((uint8_t*)&m_header)[headerIndex()] = m_byte;
        m_parserState++;
        addToTempChecksum();
    }

    inline void UBXEavesdropper::readPayload()
    {
        m_payload[payloadIndex()] = m_byte;
        m_parserState++;
        addToTempChecksum();
    }

    inline bool UBXEavesdropper::readChecksum()
    {
        switch(checksumIndex())
        {
            case 0:
                m_checksum[0] = m_tempChecksum[0];
                m_checksum[1] = m_tempChecksum[1];
                if (m_byte != m_checksum[0]) 
                {
                    resetParserAndTmpCK();
                }
                m_parserState++;
                break;
            case 1:
                resetParserAndTmpCK();
                if (m_byte == m_checksum[1]) 
                {
                    return true;
                }
                break;
            default:
                resetParserAndTmpCK();
                break;
        }
        return false;
    }

    inline uint16_t UBXEavesdropper::headerIndex() 
    {
        return m_parserState - sizeof(m_preamble);
    }

    inline uint16_t UBXEavesdropper::payloadIndex() 
    {
        return m_parserState - (sizeof(m_preamble) + sizeof(Header));
    }

    inline uint16_t UBXEavesdropper::checksumIndex() 
    {
        return m_parserState - (sizeof(m_preamble) + sizeof(Header) + m_header.msg_length);
    }
}

#endif