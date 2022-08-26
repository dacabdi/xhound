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
    bool ScreenManager::nextQueued = false;
    bool ScreenManager::previousQueued = false;

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

    void ScreenManager::queueNextScreen()
    {
        lastMoved = millis();
        nextQueued = true;
    }

    void ScreenManager::queuePreviousScreen()
    {
        lastMoved = millis();
        previousQueued = true;
    }

    void ScreenManager::nextScreen()
    {
        prevScreen = currentScreen;
        auto next = (currentScreen + 1) % screens.size();
        currentScreen = next;
        goToScreen(currentScreen);
    }

    void ScreenManager::previousScreen()
    {
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

        if(nextQueued && !previousQueued)
            nextScreen();
        if(!nextQueued && previousQueued)
            previousScreen();

        nextQueued = false;
        previousQueued = false;
    }

    void ScreenManager::goToScreen(uint8_t screenNumber)
    {
        Serial.print("Going to screen: "); Serial.println(screenNumber);
        screens[prevScreen]->clear();

        for(auto i = 0; i < screens.size(); i++)
        {
            if(i == screenNumber)
            {
                screens[i]->enable();
                screens[i]->draw();
            }
            else
            {
                screens[i]->disable();
            }
        }
    }


}