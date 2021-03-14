#include <vector>
#include <map>
#include "graphics.h"
#include "bitmaps.h"

#include "views.h"

namespace GNSS_RTK_ROVER
{
    BatteryView::BatteryView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{BATTERYVIEW_HEIGHT, BATTERYVIEW_WIDTH}), percentage(-1)
    {
        this->percentageBitmaps[-1] = battery_unknown_9x15;
        this->percentageBitmaps[0] = battery_0_9x15;
        this->percentageBitmaps[33] = battery_33_9x15;
        this->percentageBitmaps[66] = battery_66_9x15;
        this->percentageBitmaps[100] = battery_100_9x15;
    }

    void BatteryView::draw()
    {
        this->clear();
        this->canvas->printBitMap(this->position.x, this->position.y, this->percentageBitmaps[this->percentage],
            this->dimensions.width, this->dimensions.height);
    }

    void BatteryView::setPercentage(int16_t percentage)
    {
        this->percentage = percentage;
    }

    BTStatusView::BTStatusView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{BTSTATUSVIEW_HEIGHT, BTSTATUSVIEW_WIDTH}), active(false) {}

    void BTStatusView::draw()
    {
        this->clear();
        auto bitmap = this->active ? bt_on_11x15 : bt_off_11x15;
        this->canvas->printBitMap(this->position.x, this->position.y, bitmap,
            this->dimensions.width, this->dimensions.height);
    }

    void BTStatusView::setStatus(bool active)
    {
        this->active = active;
    }

    RECStatusView::RECStatusView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{RECSTATUSVIEW_HEIGHT, RECSTATUSVIEW_WIDTH}), active(false) {}

    void RECStatusView::draw()
    {
        this->clear();
        auto bitmap = this->active ? rec_20x15 : norec_20x15;
        this->canvas->printBitMap(this->position.x, this->position.y, bitmap,
            this->dimensions.width, this->dimensions.height);
    }

    void RECStatusView::setStatus(bool active)
    {
        this->active = active;
    }

    DivisionLineView::DivisionLineView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{DIVISIONLINEVIEW_HEIGHT, DIVISIONLINEVIEW_WIDTH}) {}

    void DivisionLineView::draw()
    {
        this->clear();
        this->canvas->printBitMap(this->position.x, this->position.y, division_line_h_128x1,
            this->dimensions.width, this->dimensions.height);
    }

    SolutionTypeView::SolutionTypeView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{SOLUTIONTYPEVIEW_HEIGHT, SOLUTIONTYPEVIEW_WIDTH})
    {
        this->status = NoFix;
        this->statusBitmaps[NoFix] = no_fix_48x13;
        this->statusBitmaps[TwoDFix] = twoD_fix_48x13;
        this->statusBitmaps[ThreeDFix] = threeD_fix_48x13;
        this->statusBitmaps[TimeFix] = time_fix_48x13;
        this->statusBitmaps[DGPS] = dgps_48x13;
        this->statusBitmaps[FloatRTK] = float_rtk_48x13;
        this->statusBitmaps[FixedRTK] = fixed_rtk_48x13;
    }

    void SolutionTypeView::draw()
    {
        this->clear();
        this->canvas->printBitMap(this->position.x, this->position.y, this->statusBitmaps[this->status],
            this->dimensions.width, this->dimensions.height);
    }

    void SolutionTypeView::setStatus(SolutionType status)
    {
        this->status = status;
    }

    ModeView::ModeView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{OPMODEVIEW_HEIGHT, OPMODEVIEW_WIDTH})
    {
        this->modeBitmaps[Mode::Rover] = rover_mode;
        this->modeBitmaps[Mode::Base] = base_mode;
    }

    void ModeView::draw()
    {
        this->clear();
        this->canvas->printBitMap(this->position.x, this->position.y, this->modeBitmaps[this->mode],
            this->dimensions.width, this->dimensions.height);
    }

    void ModeView::setOperationalMode(Mode mode)
    {
        this->mode = mode;
    }

    AccuracyView::AccuracyView(Canvas* can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{ACCURACYVIEW_HEIGHT, ACCURACYVIEW_WIDTH}), accuracy(10) {} // everything grater or equal 10 is considered > 9.99

    void AccuracyView::draw()
    {
        this->clear();
        if(this->accuracy > 9.99)
        {
            float_t acc = 9.99;
            this->canvas->printText("Accu > ", this->position.x, this->position.y);
            this->canvas->printFloatVariable(acc, this->position.x + 42, this->position.y);
        }
        else
        {
            this->canvas->printText("Accu = ", this->position.x, this->position.y);
            this->canvas->printFloatVariable(this->accuracy, this->position.x + 42, this->position.y);
        }
    }

    void AccuracyView::setAccuracy(float_t acc)
    {
        this->accuracy = acc;
    }

    VoltageView::VoltageView(Canvas* can, Vector2D pos) 
        : Component(can, pos, Dimensions2D{VOLTAGEVIEW_HEIGHT, VOLTAGEVIEW_WIDTH}), voltage(-1) {}

    void VoltageView::draw()
    {
        this->clear();        
        this->canvas->printText("VBat = ", this->position.x, this->position.y);
        this->canvas->printFloatVariable(this->voltage, this->position.x + 42, this->position.y);
    }

    void VoltageView::setVoltage(float_t vol)
    {
        this->voltage = vol;
    }

    LogoView::LogoView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{LOGOVIEW_HEIGHT, LOGOVIEW_WIDTH}) {}

    void LogoView::draw()
    {
        this->clear();
        this->canvas->printBitMap(this->position.x, this->position.y, logo_128x32,
            this->dimensions.width, this->dimensions.height);
    }
}