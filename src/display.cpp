#include <functional>
#include <vector>
#include <string>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "graphics.h" 
#include "display.h"

namespace GNSS_RTK_ROVER 
{
    void DisplaySSD1306::whoAmI()
    {
        Serial.println("I am Display");
    }

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

    Dimensions2D DisplaySSD1306::getDimensions()
    {
        return Dimensions2D{SSD1306_HEIGHT, SSD1306_WIDTH};
    }

    void DisplaySSD1306::display()
    {
        this->m_display.display();
    }

    void DisplaySSD1306::clear()
    {
        this->m_display.clearDisplay();
    }

    void DisplaySSD1306::printPixel(uint16_t x, uint16_t y)
    {
        m_display.setCursor(x, y);
        m_display.drawPixel(x, y, SSD1306_WHITE);
        m_display.display();
    }

    void DisplaySSD1306::erasePixel(uint16_t x, uint16_t y)
    {
        m_display.setCursor(x, y);
        m_display.drawPixel(x, y, SSD1306_BLACK);
        //m_display.display();
    }

    // Only strings less than 16 chars in size
    void DisplaySSD1306::printText(std::string text, uint16_t x, uint16_t y)
    {
        m_display.setTextSize(1);
        m_display.setTextColor(SSD1306_WHITE);
        m_display.setCursor(x, y);
        for(size_t i = 0 ; i < text.size(); i++) {
            m_display.print(text[i]);
        }
        m_display.display();
    }

    void DisplaySSD1306::printFloatVariable(float_t float_variable, uint16_t x, uint16_t y)
    {
        m_display.setTextSize(1);
        m_display.setTextColor(SSD1306_WHITE);
        m_display.setCursor(x, y);
        m_display.print(float_variable);
        m_display.display();   
    }

    // Only strings less than 16 chars in size
    void DisplaySSD1306::printTextInRect(std::string text)
    {
        m_display.clearDisplay();
        m_display.drawRect(1, 1, 127, 31, SSD1306_WHITE);
        m_display.setTextSize(1);
        m_display.setTextColor(SSD1306_WHITE);
        m_display.setCursor(64-((text.size()/2) * SSD1306_CHAR_WIDTH), 16 - SSD1306_CHAR_HEIGHT/2);
        
        for(size_t i = 0 ; i < text.size(); i++) {
            m_display.print(text[i]);
        }
        m_display.display();
        delay(2000);
        m_display.clearDisplay();
        m_display.display();
    }

    void DisplaySSD1306::printBitMap(uint16_t x, uint16_t y, const uint8_t bitmap[], uint16_t w, uint16_t h)
    {
        m_display.drawBitmap(x, y, bitmap, w, h, SSD1306_WHITE);
        m_display.display();
    }
}
