#include <vector>
#include <string>
#include <map>
#include "graphics.h"
#include "bitmaps.h"
#include "views.h"

#include "views_menu.h"

namespace GNSS_RTK_ROVER
{
    MenuOption::MenuOption(Canvas* can, Vector2D pos, std::string _text)
        : Component(can, pos, Dimensions2D{MENU_OPTION_HEIGHT, MENU_OPTION_WIDTH}), text(_text), selected(false) {}

    void MenuOption::draw()
    {
        this->clear();
        this->canvas->printText(this->text, {this->position.x, this->position.y}, this->dimensions, this->selected);
    }

    void MenuOption::select() { this->selected = true; }
    void MenuOption::unselect() { this->selected = false; }
    bool MenuOption::isSelected() { return this->selected; }

    MenuList::MenuList(Canvas* can, Vector2D pos, MenuOption* _options[], int _options_size)
        : Component(can, pos, Dimensions2D{MENU_LIST_HEIGHT, MENU_LIST_WIDTH}),
        options(_options), options_size(_options_size), viewable(MENU_LIST_VIEWABLE)
    {
        select(0);
    }

    MenuList::~MenuList()
    {
        delete[] this->options;
    }

    void MenuList::draw()
    {
        if(curr >= options_size)
        {
            Serial.println("MenuList: options empty or current out of range");
            return;
        }

        int first = curr - (this->viewable - 1);
        if(curr < this->viewable)
            first = 0;

        for(int i = 0; i < this->viewable; i++)
        {
            if(first + i >= options_size)
                break;
            options[first + i]->setPosition({this->position.x, this->position.y + (MENU_OPTION_HEIGHT * i)});
            options[first + i]->draw();
        }
    }

    void MenuList::select(int index)
    {
        if(index >= options_size)
        {
            Serial.println("MenuList: index out of range");
            return;
        }
        options[curr]->unselect();
        curr = index;
        options[curr]->select();
    }
}