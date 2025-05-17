#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define CS_PIN 5     // Chip select pin for BMP280
#define SCK_PIN 10   // Clock pin (SCK)
#define MOSI_PIN 11  // Data in (MOSI)
#define MISO_PIN 12  // Data out (MISO)

Adafruit_BMP280 bmp(CS_PIN); // Create an instance of the BMP280 sensor

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor
  Serial.println("SPI Communication with BMP280");

  // Initialize SPI with custom pins
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); // SCK, MISO, MOSI, and CS
  pinMode(CS_PIN, OUTPUT); // Set CS pin as output
  
  // Initialize the BMP280 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  Serial.println("BMP280 sensor initialized.");
}

void loop() {
  // Read data from BMP280
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure(); // Pressure in Pascals

  // Print the data
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0); // Convert pressure from Pascals to hPa
  Serial.println(" hPa");

  delay(2000); // Delay 2 seconds before next reading
}
