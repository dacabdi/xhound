#ifndef _VIEWS_MENU_H_
#define _VIEWS_MENU_H_

#include "Arduino.h"

#define MENU_OPTION_WIDTH 128
#define MENU_OPTION_HEIGHT 8

#define MENU_LIST_WIDTH 128
#define MENU_LIST_HEIGHT 18
#define MENU_LIST_VIEWABLE 2

namespace GNSS_RTK_ROVER
{
    class MenuOption : public Component
    {
        public:
        MenuOption(Canvas* can, Vector2D pos, std::string text);
        void draw() override;
        void select();
        void unselect();
        bool isSelected();

        private:
        bool selected;
        const std::string text;
    };

    typedef MenuOption* MenuOptionPtr;

    class MenuList : public Component
    {
        public:
        MenuList(Canvas* can, Vector2D pos, MenuOption* _options[], int options_size);
        ~MenuList();
        void draw() override;
        void select(int index);

        private:
        int options_size;
        int curr;
        int viewable;
        MenuOptionPtr* options;
    };
}

#endif