#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Define GPS RX/TX pins and baud rate
static const int RXPin = D2; // GPS TX -> NodeMCU D2
static const int TXPin = D1; // GPS RX -> NodeMCU D1
static const uint32_t GPSBaud = 9600;

// TinyGPS++ object
TinyGPSPlus gps;

// SoftwareSerial connection to the GPS module
SoftwareSerial ss(RXPin, TXPin);

// For stats every 5 seconds
unsigned long last = 0UL;

void setup() {
  Serial.begin(115200); // Serial Monitor
  ss.begin(GPSBaud);    // GPS communication

  Serial.println(F("GPS Initializing..."));
  Serial.println(F("Target coordinates:"));
  Serial.println(F("TARGET_LAT = 39.924840, TARGET_LON = -74.129900"));
  Serial.println(F("Verify target coordinates..."));
  Serial.println(F("Calculating descent solution..."));
}

void loop() {
  // Read from GPS module
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // If location is updated
  if (gps.location.isUpdated()) {
    Serial.print(F("Latitude: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", Longitude: "));
    Serial.println(gps.location.lng(), 6);
  }

  // Every 5 seconds, display stats and target distance
  if (millis() - last > 5000) {
    if (gps.location.isValid()) {
      static const double TARGET_LAT = 39.924840, TARGET_LON = -74.129900;
      double distanceToTarget = TinyGPSPlus::distanceBetween(
          gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);
      double courseToTarget = TinyGPSPlus::courseTo(
          gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);

      Serial.print(F("Distance to Target: "));
      Serial.print(distanceToTarget / 1000, 6);
      Serial.println(F(" km"));

      Serial.print(F("Course to Target: "));
      Serial.print(courseToTarget, 6);
      Serial.print(F(" degrees ("));
      Serial.print(TinyGPSPlus::cardinal(courseToTarget));
      Serial.println(F(")"));
    }

    Serial.print(F("Chars Processed: "));
    Serial.println(gps.charsProcessed());
    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data received. Check wiring."));

    last = millis();
  }
}