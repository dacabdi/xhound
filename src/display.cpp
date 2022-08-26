#include <functional>
#include <vector>
#include <string>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "graphics.h"
#include "display.h"

namespace GNSS_RTK_ROVER
{
    DisplaySSD1306::DisplaySSD1306(std::pair<int, int> offset, std::function<void()> onConnected, std::function<void()> onTryingConnection)
        : m_offset(offset), m_onConnected(onConnected), m_onTryingConnection(onTryingConnection)
    {
        m_display = Adafruit_SSD1306(SSD1306_WIDTH, SSD1306_HEIGHT, &Wire, -1);
        initialize();
    }

    void DisplaySSD1306::initialize()
    {
        connect();
        m_display.clearDisplay();
        m_display.display();
        m_display.setTextWrap(false);
    }

    void DisplaySSD1306::connect()
    {
        while(!m_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
            m_onTryingConnection();
            delay(100);
        }
        m_onConnected();
    }

    void DisplaySSD1306::applyOffset(Vector2D& pos)
    {
        pos.x += this->m_offset.first;
        pos.y += this->m_offset.second;
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

    void DisplaySSD1306::printPixel(Vector2D pos)
    {
        this->applyOffset(pos);
        m_display.drawPixel(pos.x, pos.y, SSD1306_WHITE);
        m_display.display();
    }

    void DisplaySSD1306::erasePixel(Vector2D pos)
    {
        this->applyOffset(pos);
        m_display.drawPixel(pos.x, pos.y, SSD1306_BLACK);
    }

    void DisplaySSD1306::erase(Vector2D pos, Dimensions2D dim)
    {
        this->applyOffset(pos);
        for(size_t i = 0; i < dim.width; i++)
            for(size_t j = 0; j < dim.height; j++)
                m_display.drawPixel(pos.x + i, pos.y + j, SSD1306_BLACK);
    }

    void DisplaySSD1306::fill(Vector2D pos, Dimensions2D dim)
    {
        this->applyOffset(pos);
        for(size_t i = 0; i < dim.width; i++)
            for(size_t j = 0; j < dim.height; j++)
                m_display.drawPixel(pos.x + i, pos.y + j, SSD1306_WHITE);
        m_display.display();
    }

    // Only strings less than 16 chars in size
    void DisplaySSD1306::printText(std::string text, Vector2D pos, Dimensions2D dim, bool highlight)
    {
        this->applyOffset(pos);
        auto color = SSD1306_WHITE;
        if(highlight)
        {
            color = SSD1306_BLACK;
            this->fill({pos.x, pos.y}, dim);
        }
        m_display.setTextSize(1);
        m_display.setTextColor(color);
        m_display.setCursor(pos.x, pos.y);
        for(size_t i = 0 ; i < text.size(); i++) {
            m_display.print(text[i]);
        }
        m_display.display();
    }

    void DisplaySSD1306::printFloatVariable(float_t float_variable, Vector2D pos)
    {
        this->applyOffset(pos);
        m_display.setTextSize(1);
        m_display.setTextColor(SSD1306_WHITE);
        m_display.setCursor(pos.x, pos.y);
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

    void DisplaySSD1306::printBitMap(Vector2D pos, Dimensions2D dim, const uint8_t bitmap[])
    {
        this->applyOffset(pos);
        m_display.drawBitmap(pos.x, pos.y, bitmap, dim.width, dim.height, SSD1306_WHITE);
        m_display.display();
    }
}
