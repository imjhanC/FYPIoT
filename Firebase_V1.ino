#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions
#include "addons/RTDBHelper.h"

// WiFi Credentials
#define WIFI_SSID "Houseofchin@unifi"
#define WIFI_PASSWORD "Happyhouse12345!"

// Firebase Credentials
#define API_KEY ""
#define DATABASE_URL "https://fypbus-e8b65-default-rtdb.asia-southeast1.firebasedatabase.app/" 

// Define GPS RX and TX pins
#define RX 14
#define TX 12
#define GPS_BAUD 9600

// Firebase Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// GPS Object
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX, TX);

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD);
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to Wi-Fi");
  
  // Firebase Configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase Sign-Up OK");
    signupOK = true;
  } else {
    Serial.printf("Firebase Sign-Up FAILED: %s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  // Read GPS data for 1 second
  unsigned long start = millis();
  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  }

  // Check if GPS data is valid
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    float altitude = gps.altitude.meters();
    float speed = gps.speed.kmph();
    int satellites = gps.satellites.value();
    float hdop = gps.hdop.hdop();
    String timestamp = String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + 
                       String(gps.date.day()) + " " + String(gps.time.hour()) + ":" + 
                       String(gps.time.minute()) + ":" + String(gps.time.second());

    Serial.println("Sending GPS Data to Firebase...");

    if (Firebase.ready() && signupOK) {
      FirebaseJson json;
      json.set("latitude", latitude);
      json.set("longitude", longitude);
      json.set("altitude", altitude);
      json.set("speed_kmph", speed);
      json.set("satellites", satellites);
      json.set("hdop", hdop);
      json.set("timestamp", timestamp);

      if (Firebase.RTDB.setJSON(&fbdo, "/gps_data", &json)) {
        Serial.println("Firebase Update SUCCESS");
      } else {
        Serial.println("Firebase Update FAILED: " + fbdo.errorReason());
      }
    }
  } else {
    Serial.println("No Valid GPS Data");
  }

  delay(5000);  // Send data every 15 seconds
}
