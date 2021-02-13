#ifndef _BITMAPS_H_
#define _BITMAPS_H_

#include "Arduino.h"

namespace GNSS_RTK_ROVER 
{
    // 'Logo', 128x32px
    const uint8_t logo_128x32 [] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x7c, 0x00, 0x00, 0x1f, 0x81, 0xc0, 0x07, 0x87, 0x11, 0x38, 0x70, 0x07, 0x3f, 0xc8, 0x80, 
	0x00, 0x3f, 0x00, 0x00, 0x1e, 0x01, 0xc0, 0x07, 0x88, 0x99, 0x45, 0x88, 0x08, 0x84, 0x09, 0x00, 
	0x00, 0x1f, 0x80, 0x00, 0x3c, 0x01, 0xc0, 0x07, 0x88, 0x99, 0x45, 0x88, 0x08, 0x84, 0x0a, 0x00, 
	0x00, 0x0f, 0xc0, 0x00, 0xf8, 0x01, 0xc0, 0x07, 0x88, 0x1d, 0x41, 0x80, 0x08, 0x84, 0x0c, 0x00, 
	0x00, 0x03, 0xe0, 0x01, 0xf0, 0x01, 0xc0, 0x07, 0x89, 0x97, 0x38, 0x73, 0xef, 0x04, 0x0c, 0x00, 
	0x00, 0x00, 0xf0, 0x03, 0xe0, 0x01, 0xc0, 0x07, 0x88, 0x93, 0x04, 0x08, 0x0c, 0x04, 0x0c, 0x00, 
	0x00, 0x00, 0x78, 0x07, 0xc0, 0x01, 0xc0, 0x07, 0x88, 0x93, 0x04, 0x08, 0x0a, 0x04, 0x0a, 0x00, 
	0x00, 0x00, 0x3c, 0x0f, 0x00, 0x01, 0xc0, 0x07, 0x88, 0x91, 0x45, 0x88, 0x09, 0x04, 0x09, 0x00, 
	0x00, 0x00, 0x1e, 0x1e, 0x00, 0x01, 0xc0, 0x07, 0x87, 0x11, 0x38, 0x70, 0x08, 0x84, 0x08, 0x80, 
	0x00, 0x00, 0x0f, 0x3c, 0x00, 0x01, 0xc0, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xb8, 0x00, 0x01, 0xc0, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xf8, 0x00, 0x01, 0xc0, 0x07, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
	0x00, 0x00, 0x01, 0xf0, 0x00, 0x01, 0xc0, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xf8, 0x00, 0x01, 0xc0, 0x07, 0x80, 0xfe, 0x18, 0x0c, 0x60, 0x30, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf8, 0x00, 0x01, 0xff, 0xff, 0x81, 0xff, 0x18, 0x0c, 0x60, 0x30, 0x01, 0x80, 
	0x00, 0x00, 0x0f, 0xbc, 0x00, 0x01, 0xff, 0xff, 0x83, 0x83, 0x98, 0x0c, 0x70, 0x30, 0x01, 0x80, 
	0x00, 0x00, 0x1f, 0x1e, 0x00, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x78, 0x30, 0x01, 0x80, 
	0x00, 0x00, 0x3e, 0x0f, 0x00, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x6c, 0x30, 0x01, 0x80, 
	0x00, 0x00, 0x7c, 0x07, 0xc0, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x66, 0x30, 0x01, 0x80, 
	0x00, 0x00, 0xf8, 0x03, 0xe0, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x63, 0x30, 0x01, 0x80, 
	0x00, 0x03, 0xf0, 0x01, 0xf0, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x61, 0xb0, 0x01, 0x80, 
	0x00, 0x0f, 0xe0, 0x00, 0xf8, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x60, 0xf0, 0x01, 0x80, 
	0x00, 0x1f, 0xc0, 0x00, 0x3c, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x60, 0x70, 0xfd, 0x80, 
	0x00, 0x3f, 0x80, 0x00, 0x1e, 0x01, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x60, 0x31, 0xc5, 0x80, 
	0x00, 0x7f, 0x00, 0x00, 0x0f, 0x81, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x60, 0x33, 0x03, 0x80, 
	0x00, 0xfc, 0x00, 0x00, 0x07, 0xc1, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x60, 0x32, 0x01, 0x80, 
	0x01, 0xf0, 0x00, 0x00, 0x03, 0xe1, 0xc0, 0x07, 0x83, 0x01, 0x98, 0x0c, 0x60, 0x32, 0x01, 0x80, 
	0x03, 0xe0, 0x00, 0x00, 0x01, 0xf1, 0xc0, 0x07, 0x83, 0x83, 0x9e, 0x3c, 0x60, 0x33, 0x03, 0x80, 
	0x0f, 0xc0, 0x00, 0x00, 0x00, 0x79, 0xc0, 0x07, 0x81, 0xff, 0x07, 0xf0, 0x60, 0x31, 0xc7, 0x80, 
	0x0f, 0x80, 0x00, 0x00, 0x00, 0x3d, 0xc0, 0x07, 0x80, 0xfe, 0x03, 0xe0, 0x60, 0x30, 0xfd, 0x80, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'division_line_v', 1x32px
    const uint8_t division_line_v [] PROGMEM = {
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
    };

    // 'clear_float_variable', 32x8px
    const uint8_t clear_float_variable [] PROGMEM = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    // 'clear_icon', 21x32px
    const uint8_t clear_icon [] PROGMEM = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    // 'clear_icon_big', 64x15px
    const uint8_t clear_icon_big [] PROGMEM = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    // 'bt_off', 21x32px
    const uint8_t bt_off [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x70, 0x00, 0x00, 0x58, 0x00, 0x00, 0x44, 0x00, 0x00, 
        0x46, 0x00, 0x00, 0x41, 0x00, 0x00, 0x41, 0x80, 0x00, 0x40, 0x40, 0x30, 0x40, 0x60, 0x38, 0x40, 
        0x60, 0x1c, 0x40, 0xc0, 0x0e, 0x41, 0x80, 0x07, 0x43, 0x00, 0x03, 0xc6, 0x00, 0x01, 0xfc, 0x00, 
        0x00, 0xfc, 0x00, 0x00, 0xfc, 0x00, 0x01, 0xfe, 0x00, 0x03, 0xc3, 0x00, 0x07, 0x41, 0x80, 0x0e, 
        0x40, 0xc0, 0x1c, 0x40, 0x60, 0x38, 0x40, 0x60, 0x30, 0x40, 0xc0, 0x00, 0x41, 0x80, 0x00, 0x43, 
        0x00, 0x00, 0x46, 0x00, 0x00, 0x4c, 0x00, 0x00, 0x58, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00
    };

    // 'bt_on', 21x32px
    const uint8_t bt_on [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x78, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x6e, 0x00, 0x00, 
        0x67, 0x00, 0x00, 0x63, 0x80, 0x00, 0x61, 0xc0, 0x00, 0x60, 0xe0, 0x30, 0x60, 0xe0, 0x38, 0x60, 
        0xe0, 0x1c, 0x61, 0xc0, 0x0e, 0x63, 0x80, 0x27, 0x67, 0x20, 0x33, 0xfe, 0x40, 0x39, 0xfc, 0xe0, 
        0x3d, 0xfd, 0x60, 0x39, 0xfc, 0xe0, 0x33, 0xfe, 0x60, 0x27, 0x67, 0x20, 0x0e, 0x63, 0x80, 0x1c, 
        0x61, 0xc0, 0x38, 0x60, 0xe0, 0x30, 0x60, 0xe0, 0x00, 0x61, 0xc0, 0x00, 0x63, 0x80, 0x00, 0x67, 
        0x00, 0x00, 0x6e, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x78, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00
    };

    // 'battery_unknown', 21x32px
    const uint8_t battery_unknown [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x0f, 
        0xff, 0x80, 0x0f, 0xff, 0x80, 0x08, 0x00, 0x80, 0x08, 0xf8, 0x80, 0x09, 0xfc, 0x80, 0x0b, 0x06, 
        0x80, 0x0b, 0x06, 0x80, 0x0b, 0x06, 0x80, 0x09, 0x06, 0x80, 0x08, 0x06, 0x80, 0x08, 0x0c, 0x80, 
        0x08, 0x18, 0x80, 0x08, 0x30, 0x80, 0x08, 0x60, 0x80, 0x08, 0x60, 0x80, 0x08, 0x60, 0x80, 0x08, 
        0x60, 0x80, 0x08, 0x60, 0x80, 0x08, 0x00, 0x80, 0x08, 0x60, 0x80, 0x08, 0x60, 0x80, 0x08, 0x00, 
        0x80, 0x08, 0x00, 0x80, 0x0f, 0xff, 0x80, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'battery_0', 21x32px
    const uint8_t battery_0 [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x0f, 
        0xff, 0x80, 0x0f, 0xff, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 
        0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 
        0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 
        0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 
        0x80, 0x08, 0x00, 0x80, 0x0f, 0xff, 0x80, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'battery_25', 21x32px
    const uint8_t battery_25 [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x0f, 
        0xff, 0x80, 0x0f, 0xff, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 
        0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 
        0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 
        0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 
        0x80, 0x08, 0x00, 0x80, 0x0f, 0xff, 0x80, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'battery_50', 21x32px
    const uint8_t battery_50 [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x0f, 
        0xff, 0x80, 0x0f, 0xff, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 
        0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 
        0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 
        0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 
        0x80, 0x08, 0x00, 0x80, 0x0f, 0xff, 0x80, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'battery_75', 21x32px
    const uint8_t battery_75 [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x0f, 
        0xff, 0x80, 0x0f, 0xff, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x08, 0x00, 
        0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 
        0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 
        0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 
        0x80, 0x08, 0x00, 0x80, 0x0f, 0xff, 0x80, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'battery_100', 21x32px
    const uint8_t battery_100 [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x0f, 
        0xff, 0x80, 0x0f, 0xff, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 
        0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 
        0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 
        0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 0x80, 0x08, 0x00, 0x80, 0x0b, 0xfe, 
        0x80, 0x08, 0x00, 0x80, 0x0f, 0xff, 0x80, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'float_rtk', 64x15px
    const uint8_t float_rtk [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x3f, 0x60, 0x7c, 0x7c, 0xfc, 0x3e, 0x7e, 0xc4, 0x3f, 0x60, 0xfe, 0xfe, 0xfc, 0x7f, 0x7e, 0xcc, 
        0x30, 0x60, 0xce, 0xce, 0x30, 0x63, 0x18, 0xcc, 0x30, 0x60, 0xc6, 0x86, 0x30, 0x63, 0x18, 0xd8, 
        0x3e, 0x60, 0xc6, 0x86, 0x30, 0x63, 0x18, 0xe0, 0x3e, 0x60, 0xc6, 0x86, 0x30, 0x63, 0x18, 0xf0, 
        0x30, 0x60, 0xc6, 0xfe, 0x30, 0x7d, 0x18, 0xf0, 0x30, 0x60, 0xc6, 0xfe, 0x30, 0x7e, 0x18, 0xf0, 
        0x30, 0x60, 0xc6, 0x86, 0x30, 0x64, 0x18, 0xd8, 0x30, 0x60, 0xce, 0x86, 0x30, 0x66, 0x18, 0xcc, 
        0x30, 0x7e, 0xfe, 0x86, 0x30, 0x63, 0x18, 0xcc, 0x30, 0x7e, 0x7c, 0x86, 0x30, 0x63, 0x18, 0xcc, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // 'fixed_rtk', 64x15px
    const uint8_t fixed_rtk [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x3f, 0x66, 0x05, 0xf9, 0xe0, 0x3e, 0x7e, 0xc4, 0x3f, 0x67, 0x0d, 0xf9, 0xb0, 0x7f, 0x7e, 0xcc, 
        0x30, 0x67, 0x1d, 0x81, 0xb8, 0x63, 0x18, 0xcc, 0x30, 0x63, 0xb9, 0x81, 0x9c, 0x63, 0x18, 0xd8, 
        0x3e, 0x61, 0xf1, 0xf1, 0x9c, 0x63, 0x18, 0xe0, 0x3e, 0x60, 0xe1, 0xf1, 0x8c, 0x63, 0x18, 0xf0, 
        0x30, 0x60, 0xe1, 0xf1, 0x8c, 0x7d, 0x18, 0xf0, 0x30, 0x60, 0xf1, 0x81, 0x9c, 0x7e, 0x18, 0xf0, 
        0x30, 0x61, 0xf9, 0x81, 0x9c, 0x64, 0x18, 0xd8, 0x30, 0x63, 0xbd, 0x81, 0xb8, 0x66, 0x18, 0xcc, 
        0x30, 0x67, 0x1d, 0xf9, 0xb0, 0x63, 0x18, 0xcc, 0x30, 0x6e, 0x0d, 0xf9, 0xe0, 0x63, 0x18, 0xcc, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    // 'dpgs', 64x15px
    const uint8_t dgps [] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x3c, 0x07, 0xc7, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x36, 0x0e, 0xe7, 0xf9, 0xf8, 0x00, 0x00, 0x00, 
        0x37, 0x14, 0x66, 0x39, 0x8c, 0x00, 0x00, 0x00, 0x33, 0x98, 0x66, 0x19, 0x8c, 0x00, 0x00, 0x00, 
        0x33, 0x98, 0x06, 0x39, 0xc4, 0x00, 0x00, 0x00, 0x31, 0x98, 0x07, 0xf8, 0xf0, 0x00, 0x00, 0x00, 
        0x31, 0x98, 0xe7, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x33, 0x98, 0xe6, 0x00, 0x18, 0x00, 0x00, 0x00, 
        0x33, 0x98, 0x26, 0x01, 0x0c, 0x00, 0x00, 0x00, 0x37, 0x1c, 0x26, 0x01, 0x8c, 0x00, 0x00, 0x00, 
        0x36, 0x0f, 0xe6, 0x01, 0xdc, 0x00, 0x00, 0x00, 0x3c, 0x07, 0xc6, 0x00, 0xf8, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    // 'base_mode', 64x15px
    const uint8_t base_mode [] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x7c, 0x7c, 0x78, 0xfc, 0x81, 0x3e, 0x78, 0x7e, 0x7e, 0xfe, 0xfc, 0xfc, 0xc3, 0x7f, 0x6c, 0x7e, 
    0x62, 0xce, 0xc6, 0xc0, 0xe7, 0x67, 0x6e, 0x60, 0x62, 0x86, 0xc6, 0xc0, 0xff, 0x63, 0x67, 0x60, 
    0x66, 0x86, 0xe2, 0xf8, 0xdd, 0x63, 0x67, 0x7c, 0x7c, 0x86, 0x78, 0xf8, 0xc9, 0x63, 0x63, 0x7c, 
    0x66, 0xfe, 0x78, 0xf8, 0xc1, 0x63, 0x63, 0x7c, 0x62, 0xfe, 0x0c, 0xc0, 0xc1, 0x63, 0x67, 0x60, 
    0x62, 0x86, 0x86, 0xc0, 0xc1, 0x63, 0x67, 0x60, 0x62, 0x86, 0xc6, 0xc0, 0xc1, 0x67, 0x6e, 0x60, 
    0x7e, 0x86, 0xee, 0xfc, 0xc1, 0x7f, 0x6c, 0x7e, 0x7c, 0x86, 0x7c, 0xfc, 0xc1, 0x3e, 0x78, 0x7e, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    // 'rover_mode', 64x15px
    const uint8_t rover_mode [] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x3e, 0x3e, 0x46, 0xfc, 0x81, 0x3e, 0x78, 0x7e, 0x7f, 0x7f, 0x46, 0xfc, 0xc3, 0x7f, 0x6c, 0x7e, 
    0x63, 0x67, 0x46, 0xc0, 0xe7, 0x67, 0x6e, 0x60, 0x63, 0x63, 0x46, 0xc0, 0xff, 0x63, 0x67, 0x60, 
    0x63, 0x63, 0x46, 0xf8, 0xdd, 0x63, 0x67, 0x7c, 0x63, 0x63, 0x46, 0xf8, 0xc9, 0x63, 0x63, 0x7c, 
    0x7d, 0x63, 0x46, 0xf8, 0xc1, 0x63, 0x63, 0x7c, 0x7e, 0x63, 0x4e, 0xc0, 0xc1, 0x63, 0x67, 0x60, 
    0x64, 0x63, 0x4c, 0xc0, 0xc1, 0x63, 0x67, 0x60, 0x66, 0x67, 0x58, 0xc0, 0xc1, 0x67, 0x6e, 0x60, 
    0x63, 0x7f, 0x70, 0xfc, 0xc1, 0x7f, 0x6c, 0x7e, 0x63, 0x3e, 0x20, 0xfc, 0xc1, 0x3e, 0x78, 0x7e, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
}

#endif