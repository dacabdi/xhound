#include <vector>
#include <string>
#include "graphics.h"

#include "screens.h"

namespace GNSS_RTK_ROVER
{
    bool ScreenManager::initialized = false;
    std::vector<Component*> ScreenManager::screens;
    int8_t ScreenManager::currentScreen = 0;
    int8_t ScreenManager::prevScreen = 0;
    int ScreenManager::lastMoved = 0;

    void ScreenManager::setup(std::vector<Component*> _screens)
    {
        initialized = true;
        screens = _screens;
        mainScreen();      
    }

    void ScreenManager::stop()
    {
        initialized = false;
    }

    void ScreenManager::mainScreen()
    {
        currentScreen = 0;
        goToScreen(currentScreen);
    }

    void ScreenManager::nextScreen()
    {
        lastMoved = millis();
        prevScreen = currentScreen;
        auto next = (currentScreen + 1) % screens.size();
        currentScreen = next;
        goToScreen(currentScreen);
    }

    void ScreenManager::previousScreen()
    {
        lastMoved = millis();
        prevScreen = currentScreen;
        currentScreen = currentScreen == 0 ? screens.size() - 1 : currentScreen - 1;
        goToScreen(currentScreen);
    }

    void ScreenManager::refresh()
    {
        if(!initialized)
            return;

        if(currentScreen != 0 && (millis() - lastMoved) > 60000)
        {
            mainScreen();
        }
    }

    void ScreenManager::goToScreen(uint8_t screenNumber)
    {
        screens[prevScreen]->clear();

        for(auto i = 0; i < screens.size(); i++)
        {
            if(screenNumber)
            {
                screens[0]->enable();
                screens[0]->draw();
            }
            else
            {
                screens[i]->disable();  
            }
        }
    }


}