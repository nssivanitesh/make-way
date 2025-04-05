#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

// Define LoRa parameters
#define LORA_SCK 5      // GPIO5 -- SX1276 SCK
#define LORA_MISO 19    // GPIO19 -- SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -- SX1276 MOSI
#define LORA_CS 18      // GPIO18 -- SX1276 CS
#define LORA_RST 14     // GPIO14 -- SX1276 RST
#define LORA_IRQ 26     // GPIO26 -- SX1276 DIO0

#define LORA_FREQUENCY 868E6 // Frequency for Sweden (EU_868 band)

// Define GPS parameters
#define GPS_RX 34       // GPIO34 -- GPS RX (Serial2 RX)
#define GPS_TX 12       // GPIO12 -- GPS TX (Serial2 TX)

// Define AXP192 parameters
#define AXP192_ADDR 0x34

// GPS instance
TinyGPSPlus gps;

// OLED display parameters
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String macAddress = "";  // Default value
unsigned long startTime = 0;             // Start time of the current hour (milliseconds)
unsigned long bytesTransmitted = 0;      // Bytes transmitted in the current hour
unsigned long messagesTransmitted = 0;   // Messages transmitted in the current hour
unsigned long usedAirTime = 0;           // Air time used until now
unsigned long lastMessageSentAt = 0;     // Timestamp when last message was sent
double dutyCycleLeft = 100;               // Duty cycle left in percentage (1% = 1.0)
double delayTime = 0.0;                   // Dynamic delay between transmissions (milliseconds)
const unsigned long HOUR_MS = 3600000;   // 1 hour in milliseconds (3600 * 1000)
const unsigned long USABLE_MS = 36000;   // 36 seconds in milliseconds (3600 * 1000)
const unsigned long MESSAGE_INTERVAL_MS = 15000; // 15 seconds in milliseconds (3600 * 1000)

/**
 * @brief Gets the elapsed time since the start of the current hour.
 * Resets hourly counters if a new hour has begun.
 * @return The elapsed time in milliseconds.
 */
unsigned long getElapsedTime() {
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - startTime;

  // Check if an hour has passed since startTime; reset if true
  if (timeElapsed >= HOUR_MS) {
    startTime = currentTime;         // Reset start time to current time
    bytesTransmitted = 0;            // Reset bytes transmitted
    messagesTransmitted = 0;         // Reset messages transmitted
    dutyCycleLeft = 100;             // Reset duty cycle to 100
    usedAirTime = 0;
    timeElapsed = 0; // Reset time elapsed
  }
  return timeElapsed;
}

/**
 * @brief Gets the remaining time until the end of the current hour.
 * @return The remaining time in milliseconds.
 */
unsigned long getTimeLeft() {
  return HOUR_MS - getElapsedTime();
}

/**
 * @brief Calculates the dynamic delay interval for the next transmission
 * based on airtime usage and remaining time.
 * @param transferTime The time taken for the current transmission (milliseconds).
 * @return The calculated delay interval in milliseconds.
 */
double getInterval(unsigned long transferTime) {
  // Update airtime and message counters
  usedAirTime += transferTime;
  messagesTransmitted += 1;

  // Get time metrics
  unsigned long timeElapsed = getElapsedTime();
  unsigned long timeLeft = getTimeLeft();
  unsigned long availableTime = (USABLE_MS > usedAirTime) ? (USABLE_MS - usedAirTime) : 0;

  // Handle edge cases: no messages or no available airtime
  if (messagesTransmitted == 0 || availableTime == 0) {
    return (double)timeLeft; // Wait until the next hour
  }

  dutyCycleLeft = (double)usedAirTime / USABLE_MS * 100;

  // Calculate the dynamic interval
  double avgTimePerMessage = (double)usedAirTime / messagesTransmitted;
  double potentialMessages = (double)availableTime / avgTimePerMessage;

  if (potentialMessages <= 0) {
    return (double)timeLeft; // No more messages possible in this hour
  }

  return (double)timeLeft / potentialMessages; // Spread messages over remaining time
}

/**
 * @brief Updates transmission statistics and calculates the next delay interval.
 * @param message The message that was transmitted.
 * @param transferTime The time taken to transmit the message.
 * @return The dynamic delay interval for the next transmission.
 */
double trackTransmission(String message, long transferTime) {
  // Update bytes transmitted (message length + estimated LoRa header)
  int payloadBytes = message.length();
  int totalBytes = payloadBytes + 13; // Add ~13 bytes for LoRa header (Meshtastic estimate)
  bytesTransmitted += totalBytes;

  return getInterval(transferTime); // Calculate and return the delay
}

/**
 * @brief Gets the GPS speed as a string, handling special cases.
 * @param gps The TinyGPSPlus instance.
 * @return The speed as a string ("0", rounded speed, or "99").
 */
String getSpeedAsString(TinyGPSPlus &gps) {
  float speed = gps.speed.mps();
  if (speed <= 0.0) {
    return "0";
  } else if (speed > 0.0 && speed <= 99.0) {
    int roundedSpeed = round(speed);
    return String(roundedSpeed);
  } else {
    return "99"; // Cap at 99
  }
}

/**
 * @brief Gets the truncated latitude and longitude as a string.
 * @param gps The TinyGPSPlus instance.
 * @param decimalPlaces The number of decimal places to truncate to.
 * @return A string in the format "latitude;longitude" or "0.00000;0.00000" if invalid.
 */
String getTruncatedLatLong(TinyGPSPlus &gps, int decimalPlaces) {
  if (!gps.location.isValid()) {
    String defaultValue = "0.";
    for (int i = 0; i < decimalPlaces; i++) {
      defaultValue += "0";
    }
    return defaultValue + ";" + defaultValue;
  }

  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  double multiplier = pow(10.0, decimalPlaces);
  double truncatedLat = floor(latitude * multiplier) / multiplier;
  double truncatedLong = floor(longitude * multiplier) / multiplier;

  char latBuffer[16];         //enough space for float
  char longBuffer[16];
  dtostrf(truncatedLat, decimalPlaces + 3, decimalPlaces, latBuffer);
  dtostrf(truncatedLong, decimalPlaces + 4, decimalPlaces, longBuffer);

  return String(latBuffer) + ";" + String(longBuffer);
}

/**
 * @brief Gets the last 4 bytes of the ESP32's MAC address as a string.
 * @return The last 4 bytes of the MAC address.
 */
String getMacLast4Bytes() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13];  // 2 hex chars per byte + colon + null terminator.  Make it bigger.
  sprintf(macStr, "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

/**
 * @brief Reads and displays the battery voltage from the AXP192.
 */
void getBatteryInfo() {
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x78); // Battery voltage register
  Wire.endTransmission(false);
  Wire.requestFrom(AXP192_ADDR, 2);
  if (Wire.available() == 2) {
    unsigned int batteryVoltage = Wire.read() << 8 | Wire.read();
    float batteryVoltageFloat = batteryVoltage * 1.1 / 1000.0;
    display.setCursor(0, 56);
    display.print("Bat: ");
    display.print(batteryVoltageFloat);
    display.println("V");
    display.display();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LilyGo T-Beam Test");

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("T-Beam Test");
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

  // Test AXP192
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
  display.println("All Tests Done");
  display.display();
  delay(2000);

  randomSeed(analogRead(0));
  macAddress = getMacLast4Bytes();
  startTime = millis(); // Initialize the start time
}

void loop() {
  // GPS Test
  display.clearDisplay();
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  if (gps.location.isValid()) {
    String latLong = getTruncatedLatLong(gps, 5);
    String gpsSpeedInMS = getSpeedAsString(gps);
    String messageToSend = macAddress + ";" + gpsSpeedInMS + ";" + latLong;

    if(millis() - lastMessageSentAt > delayTime || lastMessageSentAt == 0) {
      unsigned long txStartTime = millis();
      LoRa.beginPacket();
      LoRa.print(messageToSend);
      LoRa.endPacket();
      lastMessageSentAt = millis();
      delayTime = trackTransmission(messageToSend, lastMessageSentAt - txStartTime); // Get delay
    }    

    display.println(messageToSend);
    display.println(String(bytesTransmitted));
    display.println(String(dutyCycleLeft) + "%");
    display.print("Next Delay: ");
    display.println(String(delayTime));
  } else {
    display.println("GPS Invalid");
  }
  display.display();

  // getBatteryInfo();  //removed from the main loop to reduce I2C traffic

  delay(1000); // Use the calculated delay
}
