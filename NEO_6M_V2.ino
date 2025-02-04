#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define RX 14
#define TX 12
#define GPS_BAUD 9600
#define SERIAL_BAUD 115200

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX, TX);

void setup() {
  Serial.begin(SERIAL_BAUD);
  gpsSerial.begin(GPS_BAUD);
  Serial.println("Neo 6M GPS Advanced Monitoring");
}

void loop() {
  unsigned long start = millis();
  
  while (millis() - start < 2000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    
    if (gps.location.isUpdated()) {
      displaySatelliteInfo();
      displaySignalCharacteristics();
    }
  }
}

void displaySatelliteInfo() {
  Serial.println("\n--- Satellite Information ---");
  
  // Total Satellites
  Serial.print("Total Satellites Detected: ");
  Serial.println(gps.satellites.value());
  
  // Signal Strength and Satellite Status
  Serial.print("Satellite Signal Status: ");
  if (gps.satellites.value() > 0) {
    Serial.println("Satellites Tracked");
  } else {
    Serial.println("No Satellites Detected");
  }
}

void displaySignalCharacteristics() {
  Serial.println("\n--- Signal Characteristics ---");
  
  // Signal Quality Metrics
  Serial.print("HDOP (Horizontal Dilution of Precision): ");
  Serial.println(gps.hdop.hdop(), 2);
  
  // Precision Classification
  float hdop = gps.hdop.hdop();
  Serial.print("Signal Quality: ");
  if (hdop < 1.0) {
    Serial.println("Ideal (Extremely Precise)");
  } else if (hdop < 2.0) {
    Serial.println("Excellent");
  } else if (hdop < 5.0) {
    Serial.println("Good");
  } else if (hdop < 10.0) {
    Serial.println("Moderate");
  } else {
    Serial.println("Poor");
  }
  
  // Additional Location Data
  if (gps.location.isValid()) {
    Serial.println("\n--- Location Details ---");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Speed (km/h): ");
    Serial.println(gps.speed.kmph());
  }
}

void checkGPSHealth() {
  // Basic GPS Health Check
  Serial.println("\n--- GPS Health Check ---");
  
  if (gps.location.isValid()) {
    Serial.println("GPS Status: Active and Tracking");
  } else {
    Serial.println("GPS Status: No Valid Location");
  }
}