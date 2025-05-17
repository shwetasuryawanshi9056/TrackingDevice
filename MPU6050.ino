/*#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(15, 16);  // Initialize I2C with SDA on GPIO6 and SCL on GPIO7
  
  // Initialize MPU6050
  mpu.initialize();
  
  // Check if MPU6050 is connected
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful.");
  } else {
    Serial.println("MPU6050 connection failed.");
    while (1);  // Stop execution if MPU6050 is not connected
  }
}

void loop() {
  // Variables to hold sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
  
  // Read sensor values
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  temp = mpu.getTemperature();
  
  // Print sensor data
  Serial.print("Accel X: "); 
  Serial.print(ax);
  Serial.print("  Y: "); 
  Serial.print(ay);
  Serial.print("  Z: "); 
  Serial.println(az);
  
  Serial.print("Gyro X: "); 
  Serial.print(gx);
  Serial.print("  Y: "); 
  Serial.print(gy);
  Serial.print("  Z: "); 
  Serial.println(gz);
  
  // Convert temperature to Celsius (from raw value)
  float temperature = (temp / 340.0) + 36.53;
  Serial.print("Temperature: ");
   Serial.print(temperature);
   Serial.println(" °C");
  
  delay(3000);  // Wait for 1 second before reading again
}*/
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

MPU6050 mpu;

#define CS_PIN 5     // Chip select pin for BMP280
#define SCK_PIN 10   // Clock pin (SCK)
#define MOSI_PIN 11  // Data in (MOSI)
#define MISO_PIN 12  // Data out (MISO)

Adafruit_BMP280 bmp(CS_PIN); // Create an instance of the BMP280 sensor

void setup() {
  Serial.begin(115200);
  Wire.begin(15, 16);  // Initialize I2C with SDA on GPIO 15 and SCL on GPIO 16

  // Initialize MPU6050
  mpu.initialize();

  // Check if MPU6050 is connected
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful.");
  } else {
    Serial.println("MPU6050 connection failed.");
    while (1);  // Stop execution if MPU6050 is not connected
  }

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
  // Variables to hold MPU6050 sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;

  // Read MPU6050 sensor values
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  temp = mpu.getTemperature();

  // Print MPU6050 sensor data
  Serial.print("Accel X: "); 
  Serial.print(ax);
  Serial.print("  Y: "); 
  Serial.print(ay);
  Serial.print("  Z: "); 
  Serial.println(az);
  
  Serial.print("Gyro X: "); 
  Serial.print(gx);
  Serial.print("  Y: "); 
  Serial.print(gy);
  Serial.print("  Z: "); 
  Serial.println(gz);
  
  // Convert temperature to Celsius (from raw value)
  float temperature = (temp / 340.0) + 36.53;
  Serial.print("MPU6050 Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  // Read data from BMP280
  float bmpTemperature = bmp.readTemperature();
  float pressure = bmp.readPressure(); // Pressure in Pascals

  // Print BMP280 data
  Serial.print("BMP280 Temperature = ");
  Serial.print(bmpTemperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0); // Convert pressure from Pascals to hPa
  Serial.println(" hPa");

  delay(2000); // Delay 2 seconds before next reading
}
