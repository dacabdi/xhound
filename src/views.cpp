#include <vector>
#include <map>
#include "graphics.h"
#include "bitmaps.h"

#include "views.h"

namespace GNSS_RTK_ROVER 
{
    BatteryView::BatteryView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{BATTERYVIEW_HEIGHT, BATTERYVIEW_WIDTH}), percentage(-1)
    {
        this->percentageBitmaps[-1] = battery_unknown;
        this->percentageBitmaps[0] = battery_0;
        this->percentageBitmaps[25] = battery_25;
        this->percentageBitmaps[50] = battery_50;
        this->percentageBitmaps[75] = battery_75;
        this->percentageBitmaps[100] = battery_100;
    }

    void BatteryView::draw()
    {
        this->clear();
        this->canvas.printBitMap(this->position.x, this->position.y, this->percentageBitmaps[this->percentage],
            this->dimensions.width, this->dimensions.height);
    }

    void BatteryView::setPercentage(int16_t percentage) 
    {
        this->percentage = percentage;
    }

    BTStatusView::BTStatusView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{BTSTATUSVIEW_HEIGHT, BTSTATUSVIEW_WIDTH}), active(false) {}

    void BTStatusView::draw()
    {
        this->clear();
        auto bitmap = this->active ? bt_on : bt_off;
        this->canvas.printBitMap(this->position.x, this->position.y, bitmap,
            this->dimensions.width, this->dimensions.height);
    }

    void BTStatusView::setStatus(bool active)
    {
        this->active = active;
    }

    DivisionLineView::DivisionLineView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{DIVISIONLINEVIEW_HEIGHT, DIVISIONLINEVIEW_WIDTH}) {}

    void DivisionLineView::draw()
    {
        this->clear();
        this->canvas.printBitMap(this->position.x, this->position.y, division_line_v,
            this->dimensions.width, this->dimensions.height);
    }

    RoverStatusView::RoverStatusView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{ROVERSTATUSVIEW_HEIGHT, ROVERSTATUSVIEW_WIDTH})
    {
        this->statusBitmaps[RoverStatus::DGPS] = dgps;
        this->statusBitmaps[RoverStatus::FloatRTK] = float_rtk;
        this->statusBitmaps[RoverStatus::FixedRTK] = fixed_rtk;
    }

    void RoverStatusView::draw()
    {
        this->clear();
        this->canvas.printBitMap(this->position.x, this->position.y, this->statusBitmaps[this->status],
            this->dimensions.width, this->dimensions.height);
    }

    void RoverStatusView::setStatus(RoverStatus status)
    {
        this->status = status;
    }

    OperationalModeView::OperationalModeView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{OPMODEVIEW_HEIGHT, OPMODEVIEW_WIDTH})
    {
        this->operationalModeBitmaps[OperationalMode::Rover] = rover_mode;
        this->operationalModeBitmaps[OperationalMode::Base] = base_mode;
    }

    void OperationalModeView::draw()
    {
        this->clear();
        this->canvas.printBitMap(this->position.x, this->position.y, this->operationalModeBitmaps[this->mode],
            this->dimensions.width, this->dimensions.height);
    }

    void OperationalModeView::setOperationalMode(OperationalMode mode)
    {
        this->mode = mode;
    }

    AccuracyView::AccuracyView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{ACCURACYVIEW_HEIGHT, ACCURACYVIEW_WIDTH}), accuracy(10) {} // everything grater or equal 10 is considered > 9.99

    void AccuracyView::draw()
    {
        this->clear();
        if(this->accuracy > 9.99)
        {
            float_t acc = 9.99;
            this->canvas.printText("Accu > ", this->position.x, this->position.y);
            this->canvas.printFloatVariable(acc, this->position.x + 42, this->position.y);
        }
        else
        {
            this->canvas.printText("Accu = ", this->position.x, this->position.y);
            this->canvas.printFloatVariable(this->accuracy, this->position.x + 42, this->position.y);
        }
    }

    void AccuracyView::setAccuracy(float_t acc)
    {
        this->accuracy = acc;
    }

    VoltageView::VoltageView(Canvas& can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{VOLTAGEVIEW_HEIGHT, VOLTAGEVIEW_WIDTH}), voltage(-1) {}

    void VoltageView::draw()
    {
        this->clear();        
        this->canvas.printText("VBat = ", this->position.x, this->position.y);
        this->canvas.printFloatVariable(this->voltage, this->position.x + 42, this->position.y);
    }

    void VoltageView::setVoltage(float_t vol)
    {
        this->voltage = vol;
    }

    LogoView::LogoView(Canvas& can, Vector2D pos)
        : Component(can, pos, Dimensions2D{LOGOVIEW_HEIGHT, LOGOVIEW_WIDTH}) {}

    void LogoView::draw()
    {
        this->clear();
        this->canvas.printBitMap(this->position.x, this->position.y, logo_128x32,
            this->dimensions.width, this->dimensions.height);
    }
}