#include <WiFi.h>
#include <Wire.h>

#define STASSID "PRYM AEROSPACE"
#define STAPSK "PRYM@212"
#define MAX17048_I2C_ADDRESS 0x36

const char *ssid = STASSID;
const char *password = STAPSK;

const int RXPin = 17, TXPin = 18;
const uint32_t GPSBaud = 115200;

HardwareSerial ss(1);

void setup() {
  Wire.begin(3, 2);
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  Serial.begin(9600);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  
  setup_wifi();
  Serial.println("Setup complete.");
}

void loop() {
  Wire.beginTransmission(MAX17048_I2C_ADDRESS);
  Wire.write(0x02);
  Wire.endTransmission();
  
  Wire.requestFrom(MAX17048_I2C_ADDRESS, 2);
  uint16_t soc = (Wire.read() << 8) | Wire.read();
  if (soc > 65535) soc = 65535;
  
  float batteryLevel = (float)soc / 65535.0 * 5;
  Serial.print("Battery Level: ");
  Serial.println(batteryLevel);
  
  if (ss.available()) {
    char c = ss.read();
    Serial.write(c); // Print GPS data to serial monitor
  }

  delay(1000); // Delay for readability
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
