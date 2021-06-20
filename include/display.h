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
        DisplaySSD1306(std::pair<int, int> offset, std::function<void()> onConnected, std::function<void()> onTryingConnection);
        void initialize();

        Dimensions2D getDimensions() override;
        void display() override;
        void clear() override;
        void printPixel(Vector2D pos) override;
        void erasePixel(Vector2D pos) override;
        void erase(Vector2D pos, Dimensions2D dim) override;
        void fill(Vector2D pos, Dimensions2D dim) override;
        void printText(std::string text, Vector2D pos, Dimensions2D dim = {0, 0}, bool highlight = false) override;
        void printFloatVariable(float_t float_variable, Vector2D pos) override;
        void printTextInRect(std::string text) override;
        void printBitMap(Vector2D pos, Dimensions2D dim, const uint8_t bitmap[]) override;

        private:
        void connect();
        void applyOffset(Vector2D& pos);

        Adafruit_SSD1306 m_display;
        std::pair<int, int> m_offset;
        std::function<void()> m_onConnected;
        std::function<void()> m_onTryingConnection;
    };
}

#endif