#ifndef __MOCK_SERIAL__
#define __MOCK_SERIAL__

#include <Stream.h>

namespace GNSS_RTK_ROVER
{
    class MockStream : public arduino::Stream
    {
        public:
            MockStream(uint8_t * const buffer, size_t capacity) 
            : m_buffer(buffer),
            m_capacity(capacity),
            m_pointer(0)
            {};

            virtual size_t write(uint8_t data)
            {
                m_buffer[m_pointer++] = data;
                return m_pointer;
            };

            int available() override
            {
                return m_pointer < m_capacity;
            };

            int read() override
            {
                return m_buffer[m_pointer++];
            };

            int peek() override
            {
                return m_buffer[m_pointer];
            };

        private:
            uint8_t * const m_buffer;
            const size_t m_capacity;
            size_t m_pointer;
    };
}

#endif