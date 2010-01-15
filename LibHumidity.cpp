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

/**********************************************************
 * Initialize the sensor based on the specified type.
 **********************************************************/
LibHumidity::LibHumidity(uint8_t sensorType) {

    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);  //GND pin
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH); //VCC pin

    Wire.begin();
    readDelay = 100;
}

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return float - The relative humidity in %RH
 **********************************************************/
float LibHumidity::GetHumidity(void) {

    float humidity;

    humidity = calculateHumidity(readSensor(eRHumidityHoldCmd),
                                 readSensor(eTempHoldCmd));

    return humidity;
}

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C
 **********************************************************/
float LibHumidity::GetTemperature(void) {

    float temperature;

    temperature = calculateTemperature(readSensor(eTempHoldCmd));

    return temperature;
}

/**********************************************************
 * SetReadDelay
 *  Set the I2C Read delay from the sensor.
 *
 *  The SHT21 humidity sensor datasheet says:
 *  Parameter Resolution typ max Units
 *    14 bit      66        85      ms
 *    13 bit      33        43      ms
 *    12 Bit      17        22      ms
 *    11 bit       8        11      ms
 *    10 bit       4         6      ms
 *
 *      Measurement time
 *      (max values for -40°C
 *        125°C.)
 *      8 bit 1 3 ms
 *
 **********************************************************/
void LibHumidity::SetReadDelay(uint16_t delay) {
    readDelay = delay;
}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

uint16_t LibHumidity::readSensor(uint8_t command) {

    uint16_t result;

    Wire.beginTransmission(eSHT21Address);   //begin
    Wire.send(command);                      //send the pointer location
    delay(readDelay);
    Wire.endTransmission();                  //end

    Wire.requestFrom(eSHT21Address, 3);
    while(Wire.available() < 3) {
      ; //wait
    }

    //Store the result
    result = ((Wire.receive() >> 2) << 8);
    result += Wire.receive();

    return result;
}

float LibHumidity::calculateTemperature(uint16_t analogTempValue) {

    float st;
    float r1;
    float r2;

    st = analogTempValue;
    r1 = (0.011072 * st);
    r2 = (-2.1233E-8 * st * st);

    return (-46.8375 + r1 + r2);
}

float LibHumidity::calculateHumidity(uint16_t analogHumValue, uint16_t analogTempValue) {

    float st;
    float sRH;
    float r1;
    float r2;
    float r3;
    float r4;

    st = analogTempValue;
    sRH = analogHumValue;

    r1 = 3.8219E-2 * sRH;
    r2 = 2.5329E-4 * st;
    r3 = -3.6634E-6 * sRH * sRH;
    r4 = 8.6345E-7 * sRH * st;

    return (-7.7239 + r1 + r2 + r3 + r4);
}
