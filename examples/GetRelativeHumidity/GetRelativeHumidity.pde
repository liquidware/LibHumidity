/****************************
 * GetRelativeHumidity
 *  An example sketch that reads the sensor and prints the
 *  relative humidity to the PC's serial port
 *
 *  Tested with the SHT21-Breakout
 *  Humidity sensor from Modern Device.
 *****************************/
#include <Wire.h>
#include <LibHumidity.h>

LibHumidity humidity = LibHumidity(0);

void setup() {
  Serial.begin(9600);
}


void loop() {
  Serial.print("RHumidity: ");
  Serial.print(humidity.GetHumidity());
  Serial.print(" Temp: ");
  Serial.println(humidity.GetTemperature());
}
