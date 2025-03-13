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

// Firebase RTDB Credentials
#define API_KEY ""
#define DATABASE_URL "https://fypbus-e8b65-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Firebase Authentication (for Firestore)
#define USER_EMAIL ""    // Replace with your Firebase account email
#define USER_PASSWORD ""          // Replace with your Firebase account password
#define FIREBASE_PROJECT_ID "fypbus-e8b65"     // Your Firebase project ID (from the Firebase console)

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
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Firebase Configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  
  // Authentication configuration for Firestore
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  
  // Connect to Firebase with user credentials
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase Sign-Up OK");
    signupOK = true;
  } else {
    Serial.printf("Firebase Sign-Up FAILED: %s\n", config.signer.signupError.message.c_str());
  }
  
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
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
    
    // Format timestamp properly with leading zeros
    char timestamp[25];
    sprintf(timestamp, "%04d/%02d/%02d %02d:%02d:%02d", 
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    
    Serial.println("GPS Data:");
    Serial.print("Location: ");
    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);
    Serial.print("Altitude: ");
    Serial.println(altitude);
    Serial.print("Speed: ");
    Serial.println(speed);
    Serial.print("Satellites: ");
    Serial.println(satellites);
    Serial.print("HDOP: ");
    Serial.println(hdop);
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
    
    if (Firebase.ready() && signupOK) {
      // Send to Firebase RTDB
      sendToRTDB(latitude, longitude, altitude, speed, satellites, hdop, timestamp);
      
      // Send to Firebase Firestore
      sendToFirestore(latitude, longitude, altitude, speed, satellites, hdop, timestamp);
    }
  } else {
    Serial.println("No Valid GPS Data");
  }
  
  delay(5000);  // Send data every 5 seconds
}

void sendToRTDB(float latitude, float longitude, float altitude, float speed, int satellites, float hdop, String timestamp) {
  Serial.println("Sending GPS Data to Firebase RTDB...");
  
  FirebaseJson json;
  json.set("bus_id", "BUS_001");
  json.set("latitude", latitude);
  json.set("longitude", longitude);
  json.set("altitude", altitude);
  json.set("speed_kmph", speed);
  json.set("satellites", satellites);
  json.set("hdop", hdop);
  json.set("timestamp", timestamp);
  
  if (Firebase.RTDB.setJSON(&fbdo, "/gps_data", &json)) {
    Serial.println("RTDB Update SUCCESS");
  } else {
    Serial.println("RTDB Update FAILED: " + fbdo.errorReason());
  }
}

void sendToFirestore(float latitude, float longitude, float altitude, float speed, int satellites, float hdop, String timestamp) {
  Serial.println("Updating GPS Data in Firebase Firestore...");
  
  // Define the path to the Firestore document
  String documentPath = "bus_tracking/BUS_001";
  
  // Create a FirebaseJson object for storing data
  FirebaseJson content;
  
  // In Firestore, we need to specify the data type for each field
  content.set("fields/bus_id/stringValue", "BUS_001");
  content.set("fields/latitude/doubleValue", latitude);
  content.set("fields/longitude/doubleValue", longitude);
  content.set("fields/altitude/doubleValue", altitude);
  content.set("fields/speed_kmph/doubleValue", speed);
  content.set("fields/satellites/integerValue", satellites);
  content.set("fields/hdop/doubleValue", hdop);
  content.set("fields/timestamp/stringValue", timestamp);
  
  // Use the patchDocument method to update the existing Firestore document
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw(), "")) {
    Serial.println("Firestore Update SUCCESS");
    Serial.println(fbdo.payload().c_str());
  } else {
    Serial.println("Firestore Update FAILED: " + fbdo.errorReason());
    
    // If the document doesn't exist, try to create it
    if (fbdo.errorReason().indexOf("NOT_FOUND") >= 0) {
      Serial.println("Document not found. Creating new document...");
      if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw())) {
        Serial.println("Firestore Create SUCCESS");
      } else {
        Serial.println("Firestore Create FAILED: " + fbdo.errorReason());
      }
    }
  }
}