#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <map>
#include <vector>
#include <RTClib.h>

// Define LoRa pins
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26
#define LORA_FREQUENCY 868E6 // Sweden / EU_868 band

// Define GPS pins
#define GPS_RX 34
#define GPS_TX 12

// AXP192 address
#define AXP192_ADDR 0x34

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPS instance
TinyGPSPlus gps;

// --- Data Structures ---

struct LocationData {
  float lat;
  float lon;
  float speed; // in m/s
  unsigned long timeMs;
};

struct TransmitterInfo {
  LocationData last;
  LocationData prev;
};

std::map<String, TransmitterInfo> transmitters;

// --- Utility Functions ---

// Estimate new location based on heading and speed
void moveInDirection(float lat1, float lon1, float heading, float distanceMeters, float &newLat, float &newLon) {
  const float R = 6371000; // Earth radius in meters
  float d = distanceMeters / R;
  float hRad = radians(heading);
  float latRad = radians(lat1);
  float lonRad = radians(lon1);

  float newLatRad = asin(sin(latRad) * cos(d) + cos(latRad) * sin(d) * cos(hRad));
  float newLonRad = lonRad + atan2(sin(hRad) * sin(d) * cos(latRad), cos(d) - sin(latRad) * sin(newLatRad));

  newLat = degrees(newLatRad);
  newLon = degrees(newLonRad);
}

void updateTransmitter(String mac, float speed, float lat, float lon, unsigned long timeMs) {
  auto &info = transmitters[mac];
  info.prev = info.last;
  info.last = {lat, lon, speed, timeMs};
}

void parseLoRaMessage(String message) {
  int firstSep = message.indexOf(';');
  int secondSep = message.indexOf(';', firstSep + 1);
  if (firstSep < 0 || secondSep < 0) return;

  String mac = message.substring(0, firstSep);
  float speed = message.substring(firstSep + 1, secondSep).toFloat();
  String latlong = message.substring(secondSep + 1);
  int mid = latlong.indexOf(';');
  if (mid < 0) return;

  float lat = latlong.substring(0, mid).toFloat();
  float lon = latlong.substring(mid + 1).toFloat();

  updateTransmitter(mac, speed, lat, lon, millis());
}

struct DistanceInfo {
  String mac;
  float distance;
  unsigned long ageMs;
  bool estimated;
};

std::vector<DistanceInfo> getSortedClosest(float myLat, float myLon, unsigned long now) {
  std::vector<DistanceInfo> results;

  for (auto &pair : transmitters) {
    String mac = pair.first;
    TransmitterInfo &info = pair.second;
    float elapsed = (now - info.last.timeMs) / 1000.0;
    float estLat = info.last.lat;
    float estLon = info.last.lon;
    bool estimated = false;

    if (elapsed > 1 && info.prev.timeMs > 0) {
      float heading = TinyGPSPlus::courseTo(info.prev.lat, info.prev.lon, info.last.lat, info.last.lon);
      float distMoved = info.last.speed * elapsed;
      moveInDirection(info.last.lat, info.last.lon, heading, distMoved, estLat, estLon);
      estimated = true;
    }

    float dist = TinyGPSPlus::distanceBetween(myLat, myLon, estLat, estLon);
    results.push_back({mac, dist, now - info.last.timeMs, estimated});
  }

  std::sort(results.begin(), results.end(), [](const DistanceInfo &a, const DistanceInfo &b) {
    return a.distance < b.distance;
  });

  return results;
}

// --- Setup ---

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LilyGo T-Beam Receiver");

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed"));
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("T-Beam Booting...");
  display.display();
  delay(2000);

  // Test ESP32
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ESP32 OK");
  display.display();
  delay(1000);

  // Test GPS
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Testing GPS...");
  display.display();
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
  if (Serial2.available() > 0) {
    display.println("GPS Init OK");
  } else {
    display.println("GPS Init FAILED");
  }
  display.display();
  delay(2000);

  // Test LoRa
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Testing LoRa...");
  display.display();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LoRa Fail!");
    display.display();
    while (1);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa OK");
  display.display();
  delay(2000);

  // AXP192 power test
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Testing AXP192...");
  display.display();
  Wire.begin();
  delay(1000);
  Wire.beginTransmission(AXP192_ADDR);
  byte error = Wire.endTransmission();
  if (error == 0) {
    display.println("AXP192 OK");
  } else {
    display.print("AXP192 Err: ");
    display.print(error);
  }
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("All Tests Passed");
  display.display();
  delay(2000);
}

// --- Loop ---

void loop() {
  // LoRa receive
  if (LoRa.parsePacket()) {
    String message = "";
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }
    parseLoRaMessage(message);
  }

  // GPS update
  while (Serial2.available() > 0) gps.encode(Serial2.read());

  // Display updated every second
  if (gps.location.isValid()) {
    display.clearDisplay();
    display.setCursor(0, 0);

    float myLat = gps.location.lat();
    float myLon = gps.location.lng();
    unsigned long now = millis();
    auto closest = getSortedClosest(myLat, myLon, now);

    for (int i = 0; i < min(3, (int)closest.size()); ++i) {
      auto &c = closest[i];
      unsigned long ageSec = c.ageMs / 1000;
      unsigned long min = ageSec / 60;
      unsigned long sec = ageSec % 60;

      display.print(i + 1);
      display.print(". ");
      if (min > 0) {
        display.print(min);
        display.print(":");
        if (sec < 10) display.print("0");
      }
      display.print(sec);
      display.print(" sec (");
      display.print((int)c.distance);
      display.print(" m)");
      if (c.estimated) display.print("*");
      display.println();
    }
  }
  else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Awaiting data...");
  }
  display.display();

  delay(1000); // Refresh every second
}
