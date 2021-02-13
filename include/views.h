#ifndef _VIEWS_H_
#define _VIEWS_H_

#include "Arduino.h"

#define LOGOVIEW_HEIGHT 32
#define LOGOVIEW_WIDTH 128

#define BATTERYVIEW_HEIGHT 32
#define BATTERYVIEW_WIDTH 21 

#define BTSTATUSVIEW_HEIGHT 32
#define BTSTATUSVIEW_WIDTH 21 

#define DIVISIONLINEVIEW_HEIGHT 32
#define DIVISIONLINEVIEW_WIDTH 1

#define ROVERSTATUSVIEW_HEIGHT 15
#define ROVERSTATUSVIEW_WIDTH 64

#define OPMODEVIEW_HEIGHT 15
#define OPMODEVIEW_WIDTH 64

#define ACCURACYVIEW_HEIGHT 8
#define ACCURACYVIEW_WIDTH 72

#define VOLTAGEVIEW_HEIGHT 8
#define VOLTAGEVIEW_WIDTH 72

namespace GNSS_RTK_ROVER
{
    class BatteryView : public Component 
    {
        public:
        BatteryView(Canvas& can, Vector2D pos);
        void draw() override;
        void setPercentage(int16_t percentage);

        private:
        int16_t percentage;
        std::map<int16_t, const uint8_t*> percentageBitmaps;
    };

    class BTStatusView : public Component 
    {
        public:
        BTStatusView(Canvas& can, Vector2D pos);
        void draw() override;
        void setStatus(bool active);

        private:
        bool active;
    };

    class DivisionLineView : public Component 
    {
        public:
        DivisionLineView(Canvas& can, Vector2D pos);
        void draw() override;
    };

    enum RoverStatus
    {
        DGPS = 0,
        FloatRTK = 1,
        FixedRTK = 2
    };

    class RoverStatusView : public Component 
    {
        public:
        RoverStatusView(Canvas& can, Vector2D pos);
        void draw() override;
        void setStatus(RoverStatus status);

        private:
        RoverStatus status;
        std::map<RoverStatus, const uint8_t*> statusBitmaps;
    };

    enum OperationalMode
    {
        Rover = 0,
        Base = 1
    };

    class OperationalModeView : public Component
    {
        public:
        OperationalModeView(Canvas& can, Vector2D pos);
        void draw() override;
        void setOperationalMode(OperationalMode mode);
        
        private:
        OperationalMode mode;
        std::map<OperationalMode, const uint8_t*> operationalModeBitmaps;
    };

    class AccuracyView : public Component
    {
        public:
        AccuracyView(Canvas& can, Vector2D pos);
        void draw() override;
        void setAccuracy(float_t acc);
        
        private:
        float_t accuracy;
    };

    class VoltageView : public Component
    {
        public:
        VoltageView(Canvas& can, Vector2D pos);
        void draw() override;
        void setVoltage(float_t vol);
        
        private:
        float_t voltage;
    };

    class LogoView : public Component
    {
        public:
        LogoView(Canvas& can, Vector2D pos);
        void draw() override;
    };
}

#endif