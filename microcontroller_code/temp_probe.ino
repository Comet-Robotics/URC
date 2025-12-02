#include <OneWire.h>
#include <DallasTemperature.h>
// Pin connected to the DATA pin of DS18B20
const int SENSOR_PIN = 2;
// Setup OneWire and DallasTemperature instances
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
void setup() {
 Serial.begin(9600);
 sensors.begin(); // Initialize the sensor
}
void loop() {
 sensors.requestTemperatures(); // Request temperature data
 float tempC = sensors.getTempCByIndex(0); // Get temperature in Celsius
 float tempF = tempC * 9 / 5 + 32; // Convert to Fahrenheit
 Serial.print("Temperature: ");
 Serial.print(tempC);
 Serial.print(" °C ~ ");
 Serial.print(tempF);
 Serial.println(" °F");
 delay(1000); // Update every second
}