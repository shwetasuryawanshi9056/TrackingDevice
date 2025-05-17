#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// MPU6050 & BMP280 Setup
MPU6050 mpu;
#define CS_PIN 7
#define SCK_PIN 10
#define MOSI_PIN 11
#define MISO_PIN 12
Adafruit_BMP280 bmp(CS_PIN);

// SD Card Setup
#define SDMMC_CLK 5
#define SDMMC_CMD 4
#define SDMMC_DATA 6
#define SD_CD_PIN 46

// GNSS & RTC Pins
#define IO_RXD2 17
#define IO_TXD2 18
#define IO_GSM_PWRKEY 4
#define IO_GSM_RST 5

// WiFi Credentials
const char *ssid = "Airtel_akas_3785";
const char *password = "Air@34220";
const char *serverUrl = "http://0.0.0.0:9000";

const char *filename = "/sensordata.txt";  
HardwareSerial mySerial2(1);
unsigned long currentTime;
String gnssResponse = "", rtcResponse = "";
String latitude = "0.0", longitude = "0.0", altitude = "0.0";

// Function to send AT commands
String sendData(String command, const int timeout, boolean debug) {
    String response = "";
    mySerial2.println(command);
    long int time = millis();
    while ((time + timeout) > millis()) {
        while (mySerial2.available()) {
            char c = mySerial2.read();
            response += c;
        }
    }
    if (debug) Serial.println(response);
    return response;
}

// Parse RTC Time from AT+CCLK?
void parseCCLKResponse(String response) 
{
    Serial.println("Response" + response);
    int startIndex = response.indexOf("\"") + 1;
    int endIndex = response.lastIndexOf("\"");
    if (startIndex == 0 || endIndex == -1) return;

    response = response.substring(startIndex, endIndex);
    int tzIndex = response.indexOf("+");
    if (tzIndex != -1) response = response.substring(0, tzIndex);

    String year = response.substring(0, 2);
    String month = response.substring(3, 5);
    String day = response.substring(6, 8);
    String hour = response.substring(9, 11);
    String minute = response.substring(12, 14);
    String second = response.substring(15, 17);

    rtcResponse = "20" + year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
}

// Parse GNSS Data from AT+CGNSSINFO
void parseGNSSInfo(String response) {
    Serial.println("Response" + response);
    int startIndex = response.indexOf(":") + 1;
    if (startIndex == 0) return;

    response = response.substring(startIndex);
    response.trim();

    String values[20]; 
    int index = 0;

    while (response.length() > 0) {
        int commaIndex = response.indexOf(",");
        if (commaIndex == -1) {
            values[index++] = response;
            break;
        }
        values[index++] = response.substring(0, commaIndex);
        response = response.substring(commaIndex + 1);
    }

    if (index >= 11) {
        latitude = values[5];
        longitude = values[7];
        altitude = values[9] != "" ? values[9] : values[11];
        Serial.println("Latitude: " + latitude + " Longitude: " + longitude + " Altitude: " + altitude);
    }
}
// Function to create a new file
void createNewFile() {
    File file = SD_MMC.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to create a new file");
        return;
    }
    file.println("Latitude,Longitude,Altitude,accelX,accelY,accelZ,roll,pitch,yaw,bmpTemp,pressure"); // Write headers
    file.close();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(15, 16);  
    mySerial2.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);

    pinMode(IO_GSM_RST, OUTPUT);
    digitalWrite(IO_GSM_RST, LOW);
    pinMode(IO_GSM_PWRKEY, OUTPUT);
    digitalWrite(IO_GSM_PWRKEY, HIGH);
    delay(3000);
    digitalWrite(IO_GSM_PWRKEY, LOW);

    Serial.println("Initializing ESP32-S3 with GNSS & RTC...");
    delay(5000);

    // MPU6050 Initialization
    mpu.initialize();
    if (!mpu.testConnection()) Serial.println("MPU6050 connection failed.");

    // BMP280 Initialization
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    pinMode(CS_PIN, OUTPUT);
    if (!bmp.begin()) Serial.println("BMP280 NOT detected!");

    // SD Card Initialization
    pinMode(SD_CD_PIN, INPUT_PULLUP);
    SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);
    if (!SD_MMC.begin("/sdcard", true)) Serial.println("Card Mount Failed");

    // WiFi Setup
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");

    // Initialize AT Commands
    sendData("AT", 1000, true);
    rtcResponse = sendData("AT+CCLK?", 1000, true);
    parseCCLKResponse(rtcResponse);

    sendData("AT+CGNSSPWR=1", 1000, true);
    delay(12000);
    sendData("AT+CGNSSIPR=9600", 1000, true);
    sendData("AT+CGNSSTST=1", 1000, true);

    createNewFile();
}

/*void loop() {
    rtcResponse = sendData("AT+CCLK?", 1000, true);
    parseCCLKResponse(rtcResponse);

    gnssResponse = sendData("AT+CGNSSINFO", 1000, true);
    parseGNSSInfo(gnssResponse);

    sendDataToServer();
    //delayMicroseconds(100000);
}

void sendDataToServer() {
    if (WiFi.status() != WL_CONNECTED) return;

    // Get Sensor Data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float roll = atan2(accelY, accelZ) * 180.0 / M_PI;
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
    float yaw = atan2(gz, sqrt(gx * gx + gy * gy)) * 180.0 / M_PI;
    float bmpTemp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0;

    // Prepare JSON Data
    StaticJsonDocument<512> jsonDoc;
    jsonDoc["RTC_Time"] = rtcResponse;
    jsonDoc["Latitude"] = latitude.toFloat();
    jsonDoc["Longitude"] = longitude.toFloat();
    jsonDoc["Altitude"] = altitude.toFloat();
    jsonDoc["accelX"] = accelX;
    jsonDoc["accelY"] = accelY;
    jsonDoc["accelZ"] = accelZ;
    jsonDoc["roll"] = roll;
    jsonDoc["pitch"] = pitch;
    jsonDoc["yaw"] = yaw;
    jsonDoc["bmpTemp"] = bmpTemp;
    jsonDoc["pressure"] = pressure;

    // Convert to JSON String
    String jsonData;
    serializeJson(jsonDoc, jsonData);
    Serial.println("Sending: " + jsonData);

    // Send Data to Server
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
        SD_MMC.remove(filename);
        createNewFile();
    }
    http.end();
}*/
unsigned long lastRTCTime = 0;  // Last time RTC was updated
unsigned long lastServerSend = 0;  // Last time data was sent
const unsigned long rtcUpdateInterval = 100;  // Update RTC every 1 sec
const unsigned long serverSendInterval = 50;  // Send data every 200ms

void loop() {
    //  Update RTC every 1 second
    if (millis() - lastRTCTime >= rtcUpdateInterval) {
        rtcResponse = sendData("AT+CCLK?", 1000, true);
        parseCCLKResponse(rtcResponse);
        lastRTCTime = millis();
    }

    //  Read GNSS data continuously
    gnssResponse = sendData("AT+CGNSSINFO", 100, false);  
    parseGNSSInfo(gnssResponse);

    //  Read sensor data continuously
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float roll = atan2(accelY, accelZ) * 180.0 / M_PI;
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
    float yaw = atan2(gz, sqrt(gx * gx + gy * gy)) * 180.0 / M_PI;
    float bmpTemp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0;

    //  Send data to server every 200ms
    if (millis() - lastServerSend >= serverSendInterval) {
        sendDataToServer(accelX, accelY, accelZ, roll, pitch, yaw, bmpTemp, pressure);
        lastServerSend = millis();
    }
}

// Modified sendDataToServer() function
void sendDataToServer(float accelX, float accelY, float accelZ, float roll, float pitch, float yaw, float bmpTemp, float pressure) {
    if (WiFi.status() != WL_CONNECTED) return;

    StaticJsonDocument<512> jsonDoc;
    jsonDoc["RTC_Time"] = rtcResponse;
    jsonDoc["Latitude"] = latitude.toFloat();
    jsonDoc["Longitude"] = longitude.toFloat();
    jsonDoc["Altitude"] = altitude.toFloat();
    jsonDoc["accelX"] = accelX;
    jsonDoc["accelY"] = accelY;
    jsonDoc["accelZ"] = accelZ;
    jsonDoc["roll"] = roll;
    jsonDoc["pitch"] = pitch;
    jsonDoc["yaw"] = yaw;
    jsonDoc["bmpTemp"] = bmpTemp;
    jsonDoc["pressure"] = pressure;

    String jsonData;
    serializeJson(jsonDoc, jsonData);
    Serial.println("Sending: " + jsonData);

    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    http.POST(jsonData);
    http.end();
}
