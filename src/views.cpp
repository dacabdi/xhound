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
        this->canvas->printText("BV = ", {this->position.x, this->position.y});
        this->canvas->printFloatVariable(this->voltage, {this->position.x + 30, this->position.y});
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

        // Header
        this->clear(Header);
        this->canvas->printBitMap({this->position.x + 34, this->position.y}, {6, 56}, coordinates_text);
        this->canvas->printBitMap({this->position.x, this->position.y + 7}, {1, 128}, division_line_h_128x1);

        // Latitude
        this->clear(Latitude);
        this->canvas->printText("Lat: ", {this->position.x + 4, this->position.y + 9});
        if(!this->powerSaving)
            this->canvas->printText(this->lat.c_str(), {this->position.x + 34, this->position.y + 9});
        else
            this->canvas->printText("N/A", {this->position.x + 34, this->position.y + 9});

        // Longitude
        this->clear(Longitude);
        this->canvas->printText("Lon: ", {this->position.x + 4, this->position.y + 17});
        if(!this->powerSaving)
            this->canvas->printText(this->lon.c_str(), {this->position.x + 34, this->position.y + 17});
        else
            this->canvas->printText("N/A", {this->position.x + 34, this->position.y + 17});

        // Altitude
        String heightStr = String(this->height) + " ft";

        this->clear(Altitude);
        this->canvas->printText("Alt: ", {this->position.x + 4, this->position.y + 25});
        if(!this->powerSaving)
            this->canvas->printText(heightStr.c_str(), {this->position.x + 34, this->position.y + 25});
        else
            this->canvas->printText("N/A", {this->position.x + 34, this->position.y + 25});
    }

    void CoordinatesView::setPowerSaving(bool on)
    {
        this->powerSaving = on;
    }

    void CoordinatesView::setCoordinates(String _lat, String _lon, float _height)
    {
        lat = _lat;
        lon = _lon;
        height = _height;
    }

    void CoordinatesView::clear(CoordinatesView::ViewSection section)
    {
        switch(section)
        {
            case Header:
                this->canvas->erase(this->position, {8, 128});
                break;
            case Latitude:
                this->canvas->erase({this->position.x, this->position.y +  9}, {8, 128});
                break;
            case Longitude:
                this->canvas->erase({this->position.x, this->position.y + 17}, {8, 128});
                break;
            case Altitude:
                this->canvas->erase({this->position.x, this->position.y + 25}, {8, 128});
                break;
        }
    }

    DOPScreenView::DOPScreenView(Canvas* can, Vector2D pos)
        : Component(can, pos, Dimensions2D{DOPSCREENVIEW_HEIGHT, DOPSCREENVIEW_WIDTH}), hdop(0), vdop(0), pdop(0), powerSaving(true) {}

    void DOPScreenView::draw()
    {
        if(!enabled)
            return;

        // Header
        this->clear(Header);
        this->canvas->printBitMap({this->position.x + 15, this->position.y}, {6, 97}, dop_text);
        this->canvas->printBitMap({this->position.x, this->position.y + 7}, {1, 128}, division_line_h_128x1);

        // HDOP
        this->clear(HDOP);
        this->canvas->printText("HDOP: ", {this->position.x + 4, this->position.y + 9});
        if(!this->powerSaving)
            this->canvas->printFloatVariable(this->hdop, {this->position.x + 35, this->position.y + 9});
        else
            this->canvas->printText("N/A", {this->position.x + 35, this->position.y + 9});

        // VDOP
        this->clear(VDOP);
        this->canvas->printText("VDOP: ", {this->position.x + 4, this->position.y + 17});
        if(!this->powerSaving)
            this->canvas->printFloatVariable(this->vdop, {this->position.x + 35, this->position.y + 17});
        else
            this->canvas->printText("N/A", {this->position.x + 35, this->position.y + 17});

        // PDOP
        this->clear(PDOP);
        this->canvas->printText("PDOP: ", {this->position.x + 4, this->position.y + 25});
        if(!this->powerSaving)
            this->canvas->printFloatVariable(this->pdop, {this->position.x + 35, this->position.y + 25});
        else
            this->canvas->printText("N/A", {this->position.x + 35, this->position.y + 25});
    }

    void DOPScreenView::setPowerSaving(bool on)
    {
        this->powerSaving = on;
    }

    void DOPScreenView::setDOP(float _hdop, float _vdop, float _pdop)
    {
        hdop = _hdop;
        vdop = _vdop;
        pdop = _pdop;
    }

    void DOPScreenView::clear(DOPScreenView::ViewSection section)
    {
        switch(section)
        {
            case Header:
                this->canvas->erase(this->position, {8, 128});
                break;
            case HDOP:
                this->canvas->erase({this->position.x, this->position.y +  9}, {8, 128});
                break;
            case VDOP:
                this->canvas->erase({this->position.x, this->position.y + 17}, {8, 128});
                break;
            case PDOP:
                this->canvas->erase({this->position.x, this->position.y + 25}, {8, 128});
                break;
        }
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
        : Component(can, pos, Dimensions2D{BASEINFOVIEW_HEIGHT, BASEINFOVIEW_WIDTH}), id(0), distance(0) {}

    void BaseInfoView::draw()
    {
        if(!enabled)
            return;

        // Header
        this->clear(Header);
        this->canvas->printBitMap({this->position.x + 41, this->position.y}, {6, 42}, base_info_text);
        this->canvas->printBitMap({this->position.x, this->position.y + 7}, {1, 128}, division_line_h_128x1);

        auto idStr = this->id < 999 ? "0" + String(this->id) : String(this->id);

        // Base ID
        this->clear(BaseID);
        this->canvas->printText("Base ID: ", {this->position.x + 4, this->position.y + 12});
        if(!this->powerSaving && this->id != 0)
            this->canvas->printText(idStr.c_str(), {this->position.x + 58, this->position.y + 12});
        else
            this->canvas->printText("N/A", {this->position.x + 58, this->position.y + 12});

        // Base Distance
        auto distanceMi = String(float(distance * (6.21371 * pow(10, -6)))) + "mi";

        this->clear(Distance);
        this->canvas->printText("Distance: ", {this->position.x + 4, this->position.y + 21});
        if(!this->powerSaving && this->id != 0)
            this->canvas->printText(distanceMi.c_str(), {this->position.x + 64, this->position.y + 21});
        else
            this->canvas->printText("N/A", {this->position.x + 64, this->position.y + 21});

    }

    void BaseInfoView::setPowerSaving(bool on)
    {
        this->powerSaving = on;
    }

    void BaseInfoView::setInfo(long _id, long _distance)
    {
        id = _id;
        distance = _distance;
    }

    void BaseInfoView::clear(BaseInfoView::ViewSection section)
    {
        switch(section)
        {
            case Header:
                this->canvas->erase(this->position, {8, 128});
                break;
            case BaseID:
                this->canvas->erase({this->position.x, this->position.y +  12}, {8, 128});
                break;
            case Distance:
                this->canvas->erase({this->position.x, this->position.y + 21}, {1, 128});
                break;
        }
    }

    DeviceInfoView::DeviceInfoView(Canvas* can, Vector2D pos, String model, String sn, String btID)
        : Component(can, pos, Dimensions2D{DEVICEINFOVIEW_HEIGHT, DEVICEINFOVIEW_WIDTH}), model(model), sn(sn), btID(btID) {}

    void DeviceInfoView::draw()
    {
        if(!enabled)
            return;

        // Header
        this->clear(Header);
        this->canvas->printBitMap({this->position.x + 36, this->position.y}, {6, 52}, device_info_text);
        this->canvas->printBitMap({this->position.x, this->position.y + 7}, {1, 128}, division_line_h_128x1);

        // Model
        this->clear(Model);
        this->canvas->printText((String("Model: ") + model).c_str(), {this->position.x + 4, this->position.y + 9});

        // SN
        this->clear(SN);
        this->canvas->printText((String("SN: ") + sn).c_str(), {this->position.x + 4, this->position.y + 17});

        // BTID
        this->clear(BTID);
        this->canvas->printText((String("BTID: ") + btID).c_str(), {this->position.x + 4, this->position.y + 25});
    }

    void DeviceInfoView::clear(DeviceInfoView::ViewSection section)
    {
        switch(section)
        {
            case Header:
                this->canvas->erase(this->position, {8, 128});
                break;
            case Model:
                this->canvas->erase({this->position.x, this->position.y +  9}, {8, 128});
                break;
            case SN:
                this->canvas->erase({this->position.x, this->position.y + 17}, {8, 128});
                break;
            case BTID:
                this->canvas->erase({this->position.x, this->position.y + 25}, {8, 128});
                break;
        }
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