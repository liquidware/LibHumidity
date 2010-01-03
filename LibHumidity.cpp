/*
  LibHumidity - A Humidity Library for Arduino.

  Supported Sensor modules:
    SHT21-Breakout Module - http://www.moderndevice.com/products/sht21-humidity-sensor

  Created by Christopher Ladden at Modern Device on December 2009.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
#include <Wire.h>
#include <wiring.h>
#include "LibHumidity.h"

/******************************************************************************
 * Constructors
 ******************************************************************************/

/**
 * Initialize the sensor based on the specified type.
 */
LibHumidity::LibHumidity(uint8_t TempSensorType) {

    Wire.begin();

    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);  //GND pin
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH); //VCC pin
}

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return float - The local temperature in degrees C
 **********************************************************/
uint16_t LibHumidity::GetHumidity(void) {
    uint8_t in[3];
    float frac = 0.0;
    uint8_t bit;
    uint16_t result;

    //Ask for Temperature
    Wire.beginTransmission(eSHT21Address);   //begin
    Wire.send(eTempHoldCmd);                 //send the pointer location
    Wire.endTransmission();                  //end
    delay(100);

    Wire.requestFrom(eSHT21Address, 2);
    while(Wire.available() <= 3) {
      ; //wait
    }

    in[0] = Wire.receive();
    in[1] = Wire.receive();
    in[2] = Wire.receive();

    result = (in[0] << 8) + (in[1] >> 2);

    return result;
}
