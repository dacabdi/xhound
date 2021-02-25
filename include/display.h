#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "Arduino.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 32
#define SSD1306_CHAR_HEIGHT 8
#define SSD1306_CHAR_WIDTH 6

namespace GNSS_RTK_ROVER
{
    class DisplaySSD1306 : public Canvas
    {
        public:
        DisplaySSD1306(std::function<void()> onConnected, std::function<void()> onTryingConnection);
        void initialize();
        
        Dimensions2D getDimensions() override;
        void display() override;
        void clear() override;
        void printPixel(uint16_t x, uint16_t y) override;
        void erasePixel(uint16_t x, uint16_t y) override;
        void printText(std::string text, uint16_t x, uint16_t y) override;
        void printFloatVariable(float_t float_variable, uint16_t x, uint16_t y) override;
        void printTextInRect(std::string text) override;
        void printBitMap(uint16_t x, uint16_t y, const uint8_t bitmap[], uint16_t w, uint16_t h) override;

        void whoAmI() override;

        private:
        void connect();

        Adafruit_SSD1306 m_display;
        std::function<void()> m_onConnected;
        std::function<void()> m_onTryingConnection;
    };
}

#endif