#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define the RX and TX pins for Software Serial 2
#define RX 14
#define TX 12

#define GPS_BAUD 9600

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of Software Serial
SoftwareSerial gpsSerial(RX, TX);

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD);
  Serial.println("Software Serial started at 9600 baud rate");
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  unsigned long start = millis();

  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      // Latitude and Longitude
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: "); 
      Serial.println(gps.location.lng(), 6);
      
      // Speed in km/h
      Serial.print("SPEED (km/h) = "); 
      Serial.println(gps.speed.kmph());
      
      // Altitude in meters
      Serial.print("ALT (m) = "); 
      Serial.println(gps.altitude.meters());
      
      // Altitude in feet
      Serial.print("ALT (ft) = "); 
      Serial.println(gps.altitude.feet());
      
      // HDOP (Horizontal Dilution of Precision)
      Serial.print("HDOP = "); 
      Serial.println(gps.hdop.value() / 100.0);
      
      // Number of satellites
      Serial.print("Satellites = "); 
      Serial.println(gps.satellites.value());

      // Course (direction in degrees)
      if (gps.course.isUpdated()) {
        Serial.print("Course (degrees) = "); 
        Serial.println(gps.course.deg());
      }

      // GPS Fix Status
      if (gps.location.isValid()) {
        Serial.println("GPS Fix: Valid");
      } else {
        Serial.println("GPS Fix: Invalid");
      }

      // UTC Time
      Serial.print("Time in UTC: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));

      // Date (YYYY/MM/DD)
      Serial.print("Date: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()));

      Serial.println(""); // Blank line for readability
    }
  }
}