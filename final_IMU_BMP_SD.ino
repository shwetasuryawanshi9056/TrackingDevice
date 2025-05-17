/*#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FS.h>
#include <SD_MMC.h>

MPU6050 mpu;
#define CS_PIN 7     // Chip select pin for BMP280
#define SCK_PIN 10   // Clock pin (SCK)
#define MOSI_PIN 11  // Data in (MOSI)
#define MISO_PIN 12  // Data out (MISO)
Adafruit_BMP280 bmp(CS_PIN);

#define SDMMC_CLK 5
#define SDMMC_CMD 4
#define SDMMC_DATA 6
#define SD_CD_PIN 46

void setup() {
  Serial.begin(115200);
  Wire.begin(15, 16);  // Initialize I2C with SDA on GPIO 15 and SCL on GPIO 16

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
  }
  //Serial.println("MPU6050 connected successfully");
  // Initialize SPI for BMP280
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  pinMode(CS_PIN, OUTPUT);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  //Serial.println("BMP280 sensor initialized.");

  // Initialize SD card
  pinMode(SD_CD_PIN, INPUT_PULLUP);
  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Card Mount Failed");
    while (1);
  }
  //Serial.println("SD card mounted successfully!");
}

void logToSDCard(const String &data) {
  File file = SD_MMC.open("/sensor_data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.println(data);
  file.close();
  //Serial.println("Data written to SD card");
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  //int16_t temp;

  // Read MPU6050 sensor data
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  //temp = mpu.getTemperature();
  //float mpuTempC = (temp / 340.0) + 36.53;

  // Read BMP280 data
  float TempC = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0; // Convert Pa to hPa

  // Prepare header and data strings with fixed width formatting
  static bool headersPrinted = false;
  if (!headersPrinted) {
    Serial.println("Accel X   | Accel Y   | Accel Z | Gyro X | Gyro Y | Gyro Z | BMP Temp (C) | Pressure (hPa)");
    Serial.println("--------------------------------------------------------------------------------------------");
    headersPrinted = true;
  }

  char formattedData[150]; // Buffer to hold formatted string
  sprintf(
    formattedData,
    "%-7d|  %-7d|  %-7d|  %-7d|  %-10d|  %-10d|  %-10f|  %.2f",
    ax, ay, az, gx, gy, gz, TempC, pressure
  );

  // Log formatted data to Serial Monitor
  Serial.println(formattedData);

  // Write the same formatted data to the SD card
  logToSDCard(String(formattedData));

  delay(60000); // Wait 2 seconds before next reading
}
*/
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FS.h>
#include <SD_MMC.h>

MPU6050 mpu;
#define CS_PIN 7     // Chip select pin for BMP280
#define SCK_PIN 10   // Clock pin (SCK)
#define MOSI_PIN 11  // Data in (MOSI)
#define MISO_PIN 12  // Data out (MISO)
Adafruit_BMP280 bmp(CS_PIN);

#define SDMMC_CLK 5
#define SDMMC_CMD 4
#define SDMMC_DATA 6
#define SD_CD_PIN 46

void setup() {
    Serial.begin(115200);
    Wire.begin(15, 16);  // Initialize I2C with SDA on GPIO 15 and SCL on GPIO 16

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
    }

    // Initialize SPI for BMP280
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    pinMode(CS_PIN, OUTPUT);

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }

    // Initialize SD card
    pinMode(SD_CD_PIN, INPUT_PULLUP);
    SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("Card Mount Failed");
        while (1);
    }

    // Print headers only once
    //Serial.println("accelX,accelY,accelZ,gyroX,gyroY,gyroZ,bmpTemp,pressure");
    logToSDCard("accelX,accelY,accelZ,roll,pitch,yaw,bmpTemp,pressure");
}

void logToSDCard(const String &data) {
    File file = SD_MMC.open("/sensordata.txt", FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    file.println(data);
    file.close();
}

void loop() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Read MPU6050 sensor data
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Read BMP280 data
    float TempC = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0; // Convert Pa to hPa

    // Format data in CSV format
    char formattedData[100];
    sprintf(
        formattedData,
        "%d,%d,%d,%d,%d,%d,%.2f,%.2f",
        ax, ay, az, gx, gy, gz, TempC, pressure
    );

    // Print to Serial Monitor and SD card
    Serial.println(formattedData);
    logToSDCard(String(formattedData));

    delay(60000); // Wait 60 seconds before the next reading
}
