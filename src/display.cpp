#include <functional>
#include <string>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "display.h"

namespace GNSS_RTK_ROVER 
{
    DisplaySSD1306::DisplaySSD1306(std::function<void()> onConnected, std::function<void()> onTryingConnection)
        : m_onConnected(onConnected), m_onTryingConnection(onTryingConnection)
    {
        m_display = Adafruit_SSD1306(SSD1306_WIDTH, SSD1306_HEIGHT, &Wire, -1);
        initialize();
    }

    void DisplaySSD1306::initialize()
    {
        connect();
        m_display.clearDisplay();
        m_display.display();
    }

    void DisplaySSD1306::connect()
    {
        while(!m_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
            m_onTryingConnection();
            delay(100);
        }
        m_onConnected();
    }

    // Only strings less than 16 chars in size
    void DisplaySSD1306::printTextInRect(std::string text)
    {
        m_display.clearDisplay();
        //m_display.drawRect(9, 2, 110, 28, SSD1306_WHITE);
        m_display.drawRect(1, 1, 127, 31, SSD1306_WHITE);
        m_display.setTextSize(1);
        m_display.setTextColor(SSD1306_WHITE);
        m_display.setCursor(64-((text.size()/2) * SSD1306_CHAR_WIDTH), 16 - SSD1306_CHAR_HEIGHT/2);
        
        for(size_t i =0 ; i < text.size(); i++) {
            m_display.print(text[i]);
        }
        m_display.display();
        delay(2000);
        m_display.clearDisplay();
        m_display.display();
    }

    void DisplaySSD1306::printBitMap(std::int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, int color)
    {
        m_display.drawBitmap(x, y, bitmap, w, h, color);
        m_display.display();
    }
}
