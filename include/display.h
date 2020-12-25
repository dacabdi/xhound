#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "Arduino.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 32
#define SSD1306_CHAR_HEIGHT 8
#define SSD1306_CHAR_WIDTH 6

namespace GNSS_RTK_ROVER
{
    class DisplaySSD1306
    {
        public:
        DisplaySSD1306(std::function<void()> onConnected, std::function<void()> onTryingConnection);
        void initialize();
        void printTextInRect(std::string text);

        private:
        void connect();

        Adafruit_SSD1306 m_display;
        std::function<void()> m_onConnected;
        std::function<void()> m_onTryingConnection;
    };
}

#endif