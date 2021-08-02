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
        if(!enabled)
            return;

        this->clear();
        this->canvas->printBitMap(this->position, this->dimensions, this->percentageBitmaps[this->percentage]);
    }

    void BatteryView::setPercentage(int16_t percentage)
    {
        this->percentage = percentage;
    }

    BTStatusView::BTStatusView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{BTSTATUSVIEW_HEIGHT, BTSTATUSVIEW_WIDTH}), active(false) {}

    void BTStatusView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        auto bitmap = this->active ? bt_on_11x15 : bt_off_11x15;
        this->canvas->printBitMap(this->position, this->dimensions, bitmap);
    }

    void BTStatusView::setStatus(bool active)
    {
        this->active = active;
    }

    RECStatusView::RECStatusView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{RECSTATUSVIEW_HEIGHT, RECSTATUSVIEW_WIDTH}), active(false) {}

    void RECStatusView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        auto bitmap = this->active ? rec_20x15 : norec_20x15;
        this->canvas->printBitMap(this->position, this->dimensions, bitmap);
    }

    void RECStatusView::setStatus(bool active)
    {
        this->active = active;
    }

    DivisionLineView::DivisionLineView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{DIVISIONLINEVIEW_HEIGHT, DIVISIONLINEVIEW_WIDTH}) {}

    void DivisionLineView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        this->canvas->printBitMap(this->position, this->dimensions, division_line_h_128x1);
    }

    SolutionTypeView::SolutionTypeView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{SOLUTIONTYPEVIEW_HEIGHT, SOLUTIONTYPEVIEW_WIDTH})
    {
        this->status = NoFix;
        this->statusBitmaps[GnssOff] = gnss_off_48x13;
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
        if(!enabled)
            return;

        this->clear();
        this->canvas->printBitMap(this->position, this->dimensions, this->statusBitmaps[this->status]);
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
        if(!enabled)
            return;

        this->clear();
        this->canvas->printBitMap(this->position, this->dimensions, this->modeBitmaps[this->mode]);
    }

    void ModeView::setOperationalMode(Mode mode)
    {
        this->mode = mode;
    }

    AccuracyView::AccuracyView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{ACCURACYVIEW_HEIGHT, ACCURACYVIEW_WIDTH}), accuracy(10) {} // everything grater or equal 10 is considered > 9.99

    void AccuracyView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        if(this->accuracy > 9.99)
        {
            float_t acc = 9.99;
            this->canvas->printText("Accu > ", {this->position.x, this->position.y});
            this->canvas->printFloatVariable(acc, {this->position.x + 42, this->position.y});
        }
        else
        {
            this->canvas->printText("Accu = ", {this->position.x, this->position.y});
            this->canvas->printFloatVariable(this->accuracy, {this->position.x + 42, this->position.y});
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
        if(!enabled)
            return;

        this->clear();
        this->canvas->printText("VBat = ", {this->position.x, this->position.y});
        this->canvas->printFloatVariable(this->voltage, {this->position.x + 42, this->position.y});
    }

    void VoltageView::setVoltage(float_t vol)
    {
        this->voltage = vol;
    }

    CoordinatesView::CoordinatesView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{COORDINATESVIEW_HEIGHT, COORDINATESVIEW_WIDTH}), lat(0), lon(0), height(0), powerSaving(true) {}

    void CoordinatesView::draw()
    {
        if(!enabled)
            return;

        this->clear();

        String heightStr = String(this->height) + " ft";
        
        this->canvas->printText("Lat: ", {this->position.x, this->position.y});
        if(!this->powerSaving)
            this->canvas->printText(this->lat.c_str(), {this->position.x + 30, this->position.y});
        else
            this->canvas->printText("N/A", {this->position.x + 30, this->position.y});

        this->canvas->printText("Lon: ", {this->position.x, this->position.y + 12});
        if(!this->powerSaving)
            this->canvas->printText(this->lon.c_str(), {this->position.x + 30, this->position.y + 12});
        else
            this->canvas->printText("N/A", {this->position.x + 30, this->position.y + 12});

        this->canvas->printText("Alt: ", {this->position.x, this->position.y + 24});
        if(!this->powerSaving)
            this->canvas->printText(heightStr.c_str(), {this->position.x + 30, this->position.y + 24});
        else
            this->canvas->printText("N/A", {this->position.x + 30, this->position.y + 24});
    }

    void CoordinatesView::setPowerSaving(bool on)
    {
        this->powerSaving = on;
    }

    void CoordinatesView::setCoordinates(String _lat, String _lon, long _height)
    {
        lat = _lat;
        lon = _lon;
        height = _height;
    }


    SIVView::SIVView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{SIVVIEW_HEIGHT, SIVVIEW_WIDTH}), siv(0), powerSaving(true) {}

    void SIVView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        this->canvas->printText("SIV:", {this->position.x, this->position.y});
        if(!this->powerSaving)
            this->canvas->printText(std::to_string(this->siv), {this->position.x + 30, this->position.y});
        else
            this->canvas->printText("N/A", {this->position.x + 30, this->position.y});
    }

    void SIVView::setPowerSaving(bool on)
    {
        this->powerSaving = on;
    }

    void SIVView::setSIV(int _siv)
    {
        siv = _siv;
    }

    DOPView::DOPView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{DOPVIEW_HEIGHT, DOPVIEW_WIDTH}), dop(0), powerSaving(true) {}

    void DOPView::draw()
    {
        if(!enabled)
            return;

        Serial.print("DOP is: ");
        Serial.println(this->dop);
        this->clear();
        this->canvas->printText("DOP:", {this->position.x, this->position.y});
        if(!this->powerSaving)
            this->canvas->printText(String(this->dop).c_str(), {this->position.x + 30, this->position.y});
        else
            this->canvas->printText("N/A", {this->position.x + 30, this->position.y});
    }

    void DOPView::setPowerSaving(bool on)
    {
        this->powerSaving = on;
    }

    void DOPView::setDOP(float _dop)
    {
        dop = _dop;
    }

    BaseInfoView::BaseInfoView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{COORDINATESVIEW_HEIGHT, COORDINATESVIEW_WIDTH}), id(0), distance(0) {}

    void BaseInfoView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        this->canvas->printText("Base ID: ", {this->position.x, this->position.y});
        this->canvas->printFloatVariable(this->id, {this->position.x + 54, this->position.y});
        this->canvas->printText("Distance: ", {this->position.x, this->position.y + 12});
        this->canvas->printFloatVariable(this->distance, {this->position.x + 60, this->position.y + 12});
    }

    void BaseInfoView::setInfo(long _id, long _distance)
    {
        id = _id;
        distance = _distance;
    }

    LogoView::LogoView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{LOGOVIEW_HEIGHT, LOGOVIEW_WIDTH}) {}

    void LogoView::draw()
    {
        if(!enabled)
            return;

        this->clear();
        this->canvas->printBitMap(this->position, this->dimensions, logo_128x32);
    }
}