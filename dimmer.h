/*
 * This file is part of Dimmer.
 *
 * Dimmer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Dimmer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Dimmer.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <inttypes.h>

class Dimmer
{
public:
    Dimmer (uint8_t channel, uint16_t max = 0);
    ~Dimmer();

    bool setLevel(uint16_t value);

    operator uint16_t()
    {
        return m_value;
    }

private:
    void reset();

    static bool m_configured;
    static const uint8_t PCA9685         = 0x40;
    static const uint8_t READ            = 0x01;
    static const uint8_t WRITE           = 0x00;

    static const uint8_t PCA9685_SUBADR1 = 0x02;
    static const uint8_t PCA9685_SUBADR2 = 0x03;
    static const uint8_t PCA9685_SUBADR3 = 0x04;

    static const uint8_t MODE1           = 0x00;
    static const uint8_t MODE2           = 0x01;

    static const uint8_t MODE1_RESTART    = (1 << 7);
    static const uint8_t MODE1_EXTCLK     = (1 << 6);
    static const uint8_t MODE1_AUTOINC    = (1 << 5);
    static const uint8_t MODE1_SLEEP      = (1 << 4);
    static const uint8_t MODE1_RESP_SUB1  = (1 << 3);
    static const uint8_t MODE1_RESP_SUB2  = (1 << 2);
    static const uint8_t MODE1_RESP_SUB3  = (1 << 1);
    static const uint8_t MODE1_RESP_ACALL = (1 << 0);

    static const uint8_t MODE2_INVERT     = (1 << 4);
    static const uint8_t MODE2_OCH        = (1 << 3);
    static const uint8_t MODE2_OUTDRV     = (1 << 2);
    static const uint8_t MODE2_LEDn0      = (0);
    static const uint8_t MODE2_LEDn1      = (1);
    static const uint8_t MODE2_LEDnHighZ  = (2);

    static const uint8_t PCA9685_PRESCALE = 0xFE;

    static const uint8_t LED_ON_L  = 0x06;
    static const uint8_t LED_ON_H  = 0x07;
    static const uint8_t LED_OFF_L = 0x08;
    static const uint8_t LED_OFF_H = 0x09;

    // the actual channel we are operating as
    const uint8_t m_channel;
    // offset used for our start time
    const uint16_t m_offset;

    // indexes into required channel on and off registers
    const uint8_t ON_L;
    const uint8_t ON_H;
    const uint8_t OFF_L;
    const uint8_t OFF_H;

    void setFrequency(float freq);

    uint8_t write(uint8_t addr, uint8_t * values, uint8_t len = 1);
    uint8_t read (uint8_t addr);

    // last known set value
    // this can be anything between 0 and 4095
    uint16_t m_value;
};
