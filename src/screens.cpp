#include <vector>
#include "graphics.h"

#include "screens.h"

namespace GNSS_RTK_ROVER
{
    bool ScreenManager::initialized = false;
    std::vector<Component*> ScreenManager::screens;
    int8_t ScreenManager::currentScreen = 0;
    int ScreenManager::lastMoved = 0;

    void ScreenManager::setup(std::vector<Component*> _screens)
    {
        screens = _screens;
        initialized = true;
    }

    void ScreenManager::stop()
    {
        initialized = false;
    }

    void ScreenManager::mainScreen()
    {
        currentScreen = 0;
    }

    void ScreenManager::nextScreen()
    {
        lastMoved = millis();
        auto next = (currentScreen + 1) % screens.size();
        currentScreen = next;
    }

    void ScreenManager::previousScreen()
    {
        lastMoved = millis();
        auto prev = (currentScreen - 1) % screens.size();
        currentScreen = prev;
    }

    void ScreenManager::refresh()
    {
        if(!initialized)
            return;
         
        screens[currentScreen]->clear();
        if(currentScreen != 0 && (millis() - lastMoved) > 60000)
        {
            currentScreen = 0;
        }
         screens[currentScreen]->draw();
    }
    
}