#ifndef _GRAPHICS_H_
#define _GRAPHICS_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER
{
    struct Vector2D
    {
        uint16_t x;
        uint16_t y;
    };

    struct Dimensions2D
    {
        uint16_t height;
        uint16_t width;
    };

    class Canvas
    {
        public:
        virtual ~Canvas() {};
        virtual Dimensions2D getDimensions() = 0;
        virtual void display() = 0;
        virtual void clear() = 0;
        virtual void printPixel(Vector2D pos) = 0;
        virtual void erasePixel(Vector2D pos) = 0;
        virtual void erase(Vector2D pos, Dimensions2D dim) = 0;
        virtual void fill(Vector2D pos, Dimensions2D dim) = 0;
        virtual void printText(std::string text, Vector2D pos, Dimensions2D dim = {0, 0}, bool highlight = false) = 0;
        virtual void printFloatVariable(std::float_t float_variable, Vector2D pos) = 0;
        virtual void printTextInRect(std::string text) = 0;
        virtual void printBitMap(Vector2D pos, Dimensions2D dim, const uint8_t bitmap[]) = 0;
    };

    class Component
    {
        public:
        Component() {}
        Component(Canvas* can, Vector2D pos, Dimensions2D dim);
        virtual void draw() = 0;
        void clear();

        Vector2D getPosition();
        Dimensions2D getDimensions();
        void setPosition(Vector2D pos);
        void setDimensions(Dimensions2D dim);

        protected:
        Canvas* canvas;
        Vector2D position;
        Dimensions2D dimensions;
    };

    class CompositeComponent : public Component
    {
        public:
        CompositeComponent(Canvas* can, Vector2D pos, Dimensions2D dim) : Component(can, pos, dim) {}
        void draw() override;
        void embed(Component* component);

        private:
        std::vector<Component*> subcomponents;
    };
}

#endif