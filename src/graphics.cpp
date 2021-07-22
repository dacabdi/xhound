#include <vector>
#include <string>

#include "graphics.h"

namespace GNSS_RTK_ROVER
{
    Component::Component(Canvas* can, Vector2D pos, Dimensions2D dim) : canvas(can), position(pos), dimensions(dim), enabled(true) {}

    void Component::enable()
    {
        this->enabled = true;
    }

    void Component::disable()
    {
        this->enabled = false;
    }

    void Component::clear()
    {
        this->canvas->erase(this->position, this->dimensions);
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

    void CompositeComponent::enable()
    {
        this->enabled = true;

        for(auto it = this->subcomponents.begin(); it != this->subcomponents.end(); it++)
        {
            (*it)->enable();
        }
    }

    void CompositeComponent::disable()
    {
        this->enabled = false;

        for(auto it = this->subcomponents.begin(); it != this->subcomponents.end(); it++)
        {
            (*it)->disable();
        }
    }

    void CompositeComponent::draw()
    {
        if(!enabled)
            return;

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