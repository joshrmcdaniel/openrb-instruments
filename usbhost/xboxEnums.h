/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _xboxenums_h
#define _xboxenums_h

#include "controllerEnums.h"

// PID and VID of the different versions of the controller - see:
// https://github.com/torvalds/linux/blob/c2da8b3f914f83fb9089d26a692eb8f22146ddb9/drivers/input/joystick/xpad.c#L129

// Licensed controller vendors
#define XBOX_VID1 0x045E  // Microsoft Corporation
#define XBOX_VID2 0x0738  // Mad Catz
#define XBOX_VID3 0x0E6F  // Afterglow
#define XBOX_VID4 0x0F0D  // HORIPAD ONE
#define XBOX_VID5 0x1532  // Razer
#define XBOX_VID6 0x24C6  // PowerA
#define XBOX_VID7 0x1430  // RedOctane

/** Enum used to set special LED modes supported by the Xbox controller. */
enum LEDModeEnum {
        ROTATING = 0x0A,
        FASTBLINK = 0x0B,
        SLOWBLINK = 0x0C,
        ALTERNATING = 0x0D,
};

/** Used to set the LEDs on the controllers */
const uint8_t XBOX_LEDS[] PROGMEM = {
        0x00, // OFF
        0x02, // LED1
        0x03, // LED2
        0x04, // LED3
        0x05, // LED4
        0x01, // ALL - Used to blink all LEDs
};
/** Buttons on the controllers */
const uint16_t XBOX_BUTTONS[] PROGMEM = {
        0x0100, // UP
        0x0800, // RIGHT
        0x0200, // DOWN
        0x0400, // LEFT

        0x2000, // BACK
        0x1000, // START
        0x4000, // L3
        0x8000, // R3

        0, 0, // Skip L2 and R2 as these are analog buttons
        0x0001, // L1
        0x0002, // R1

        0x0020, // B
        0x0010, // A
        0x0040, // X
        0x0080, // Y

        0x0004, // XBOX
        0x0008, // SYNC
};

inline int8_t getButtonIndexXbox(ButtonEnum b) {
        const int8_t index = ButtonIndex(b);
        if ((uint8_t) index >= (sizeof(XBOX_BUTTONS) / sizeof(XBOX_BUTTONS[0]))) return -1;
        return index;
}

#endif
