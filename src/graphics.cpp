#include "vector"
#include <string>

#include "graphics.h"

namespace GNSS_RTK_ROVER
{
    void Canvas::whoAmI()
    {
        Serial.println("I am Canvas");
    }

    Component::Component(Canvas* can, Vector2D pos, Dimensions2D dim) : canvas(can), position(pos), dimensions(dim) {}

    void Component::clear()
    {
        for(auto i = 0; i < this->dimensions.width; i++)
        {
            for(auto j = 0; j < this->dimensions.height; j++)
            {
                this->canvas->erasePixel(i + this->position.x, j + this->position.y);
            }
        }
    }

    Vector2D Component::getPosition() 
    { 
        return this->position; 
    }
    
    Dimensions2D Component::getDimensions() 
    { 
        return this->dimensions; 
    }
    
    void Component::setPosition(Vector2D pos) 
    { 
        this->position = pos; 
    }
    
    void Component::setDimensions(Dimensions2D dim) 
    { 
        this->dimensions = dim; 
    }

    void CompositeComponent::draw()
    {
        for(auto it = this->subcomponents.begin(); it != this->subcomponents.end(); it++)
        {
            (*it)->draw();
        }
    }

    void CompositeComponent::embed(Component* component)
    {
        auto pos = component->getPosition();
        pos.x += this->position.x;
        pos.y += this->position.y;
        component->setPosition(pos);
        this->subcomponents.push_back(component);
    }
}