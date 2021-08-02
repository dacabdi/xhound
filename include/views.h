#ifndef _VIEWS_H_
#define _VIEWS_H_

#include "Arduino.h"

#define LOGOVIEW_WIDTH 128
#define LOGOVIEW_HEIGHT 32

#define BATTERYVIEW_WIDTH 9
#define BATTERYVIEW_HEIGHT 15

#define BTSTATUSVIEW_WIDTH 11
#define BTSTATUSVIEW_HEIGHT 15

#define RECSTATUSVIEW_WIDTH 20
#define RECSTATUSVIEW_HEIGHT 15

#define DIVISIONLINEVIEW_WIDTH 128
#define DIVISIONLINEVIEW_HEIGHT 1

#define SOLUTIONTYPEVIEW_WIDTH 48
#define SOLUTIONTYPEVIEW_HEIGHT 13

#define OPMODEVIEW_WIDTH 64
#define OPMODEVIEW_HEIGHT 15

#define ACCURACYVIEW_WIDTH 72
#define ACCURACYVIEW_HEIGHT 8

#define VOLTAGEVIEW_WIDTH 72
#define VOLTAGEVIEW_HEIGHT 8

#define SIVVIEW_WIDTH 64
#define SIVVIEW_HEIGHT 10

#define DOPVIEW_WIDTH 64
#define DOPVIEW_HEIGHT 10

#define COORDINATESVIEW_WIDTH 120
#define COORDINATESVIEW_HEIGHT 30

#define COORDINATESVIEW_WIDTH 128
#define COORDINATESVIEW_HEIGHT 32

namespace GNSS_RTK_ROVER
{
    class BatteryView : public Component
    {
        public:
        BatteryView() {}
        BatteryView(Canvas* can, Vector2D pos);
        void draw() override;
        void setPercentage(int16_t percentage);

        private:
        int16_t percentage;
        std::map<int16_t, const uint8_t*> percentageBitmaps;
    };

    class BTStatusView : public Component
    {
        public:
        BTStatusView() {}
        BTStatusView(Canvas* can, Vector2D pos);
        void draw() override;
        void setStatus(bool active);

        private:
        bool active;
    };

    class RECStatusView : public Component
    {
        public:
        RECStatusView() {}
        RECStatusView(Canvas* can, Vector2D pos);
        void draw() override;
        void setStatus(bool active);

        private:
        bool active;
    };

    class DivisionLineView : public Component
    {
        public:
        DivisionLineView() {}
        DivisionLineView(Canvas* can, Vector2D pos);
        void draw() override;
    };


    class SolutionTypeView : public Component
    {
        public:
        enum SolutionType
        {
            GnssOff,
            NoFix,
            TwoDFix,
            ThreeDFix,
            TimeFix,
            DGPS,
            FloatRTK,
            FixedRTK
        };

        SolutionTypeView() {}
        SolutionTypeView(Canvas* can, Vector2D pos);
        void draw() override;
        void setStatus(SolutionType status);

        private:
        SolutionType status;
        std::map<SolutionType, const uint8_t*> statusBitmaps;
    };

    class ModeView : public Component
    {
        public:
        enum Mode
        {
            Rover,
            Base
        };

        ModeView() {}
        ModeView(Canvas* can, Vector2D pos);
        void draw() override;
        void setOperationalMode(Mode mode);

        private:
        Mode mode;
        std::map<Mode, const uint8_t*> modeBitmaps;
    };

    class AccuracyView : public Component
    {
        public:
        AccuracyView() {}
        AccuracyView(Canvas* can, Vector2D pos);
        void draw() override;
        void setAccuracy(float_t acc);

        private:
        float_t accuracy;
    };

    class VoltageView : public Component
    {
        public:
        VoltageView() {}
        VoltageView(Canvas* can, Vector2D pos);
        void draw() override;
        void setVoltage(float_t vol);

        private:
        float_t voltage;
    };

    class CoordinatesView : public Component
    {
        public:
        CoordinatesView() {}
        CoordinatesView(Canvas* can, Vector2D pos);
        void draw() override;
        void setPowerSaving(bool on);
        void setCoordinates(String _lat, String _lon, long _height);

        private:
        bool powerSaving;
        String lat;
        String lon;
        long height;
    };

    class SIVView : public Component
    {
        public:
        SIVView() {}
        SIVView(Canvas* can, Vector2D pos);
        void draw() override;
        void setPowerSaving(bool on);
        void setSIV(int _siv);

        private:
        bool powerSaving;
        int siv;
    };

    class DOPView : public Component
    {
        public:
        DOPView() {}
        DOPView(Canvas* can, Vector2D pos);
        void draw() override;
        void setPowerSaving(bool on);
        void setDOP(float _siv);

        private:
        bool powerSaving;
        float dop;
    };

    class BaseInfoView : public Component
    {
        public:
        BaseInfoView() {}
        BaseInfoView(Canvas* can, Vector2D pos);
        void draw() override;
        void setPowerSaving(bool on);
        void setInfo(long id, long distance);

        private:
        bool powerSaving;
        long id;
        long distance;
    };

    class LogoView : public Component
    {
        public:
        LogoView() {}
        LogoView(Canvas* can, Vector2D pos);
        void draw() override;
    };
}

#endif