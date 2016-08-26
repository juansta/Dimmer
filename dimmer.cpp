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

#include <Arduino.h>
#include <math.h>
#include <dimmer.h>
//#include <twi.h>
#include <Wire.h>

#define SDA_PIN 5
#define SCL_PIN 4

bool Dimmer::m_configured = false;

Dimmer::Dimmer(uint8_t channel, uint16_t max)
    : m_channel(channel),
      m_offset (channel * 256),
      ON_L     (channel * 4 + LED_ON_L),
      ON_H     (channel * 4 + LED_ON_H),
      OFF_L    (channel * 4 + LED_OFF_L),
      OFF_H    (channel * 4 + LED_OFF_H),
      m_value  (max)
{
    Wire.begin(SDA_PIN, SCL_PIN);

    Wire.beginTransmission(PCA9685);
    Wire.write(MODE1);
    Wire.write(MODE1_AUTOINC);
    Wire.write(MODE2_INVERT);
    Wire.endTransmission();

    setFrequency(120);
}

Dimmer::~Dimmer()
{}

void Dimmer::reset()
{
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    Wire.endTransmission();
}

bool Dimmer::setLevel(uint16_t value)
{
    // our ON values are always offset depending on channel
    // this ensures that all channels at least turn ON at different
    // times, reducing inrush requirements
    uint16_t  offset = (value + m_offset) % 4096;
    uint8_t * poff   = (uint8_t*)&offset;
    uint8_t * pon    = (uint8_t*)&m_offset;
    uint8_t   pwm[4] = {pon[0], pon[1], poff[0], poff[1]};

    bool ret = write(ON_L, pwm, 4) == 0;

    if (ret)
        m_value = value;

    return ret;
}

void Dimmer::setFrequency(float freq)
{
    float prescaleval = 25000000;

    prescaleval /= 4096;
    prescaleval /= (freq * 0.9);
    prescaleval -= 1;

    uint8_t prescale = floor(prescaleval + 0.5);
    uint8_t oldmode  = read(MODE1);
    uint8_t newmode  = (oldmode & 0x7F) | 0x10; // sleep

    write(MODE1, &newmode);             // go to sleep
    write(PCA9685_PRESCALE, &prescale); // set the prescaler
    write(MODE1, &oldmode);

    oldmode = oldmode | 0xA1;

    write(MODE1, &oldmode);
}

uint8_t Dimmer::read(uint8_t addr)
{
    Wire.beginTransmission(PCA9685);
    Wire.write(addr);
    Wire.endTransmission(false);

    Wire.requestFrom(PCA9685, (uint8_t)1);
    return Wire.read();
}

uint8_t Dimmer::write(uint8_t addr, uint8_t *values, uint8_t len)
{
    Wire.beginTransmission(PCA9685);

    Wire.write(addr);
    Wire.write(values, len);

    return Wire.endTransmission();
}
