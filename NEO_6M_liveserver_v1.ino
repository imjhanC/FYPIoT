#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define RX 14
#define TX 12
#define GPS_BAUD 9600

const char *ssid = "ria ria 327_2.4GHZ";
const char *password = ""; 

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX, TX);
ESP8266WebServer server(80);

float latitude = 0.0, longitude = 0.0, speed_kmh = 0.0;

void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html><head><title>GPS Tracker</title>";
  html += "<script src='https://unpkg.com/leaflet/dist/leaflet.js'></script>";
  html += "<link rel='stylesheet' href='https://unpkg.com/leaflet/dist/leaflet.css'/>";
  html += "<style>#map { height: 100vh; }</style></head><body>";
  html += "<div id='map'></div>";
  html += "<script>var map = L.map('map').setView([" + String(latitude, 6) + ", " + String(longitude, 6) + "], 15);";
  html += "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);";
  html += "var marker = L.marker([" + String(latitude, 6) + ", " + String(longitude, 6) + "]).addTo(map);";
  html += "var speedLabel = L.divIcon({ className: 'speed-label', html: '<b>Speed: " + String(speed_kmh, 2) + " km/h</b>', iconSize: [100, 40] });";
  html += "var speedMarker = L.marker([" + String(latitude, 6) + ", " + String(longitude, 6) + "], { icon: speedLabel }).addTo(map);";
  html += "function updateLocation() {fetch('/gps').then(response => response.json()).then(data => {";
  html += "marker.setLatLng([data.lat, data.lng]);map.setView([data.lat, data.lng]);";
  html += "speedMarker.setLatLng([data.lat, data.lng]);";
  html += "speedMarker.setIcon(L.divIcon({ className: 'speed-label', html: '<b>Speed: ' + data.speed + ' km/h</b>', iconSize: [100, 40] }));";
  html += "});}";
  html += "setInterval(updateLocation, 2000);</script>";
  html += "<style>.speed-label { background: white; padding: 5px; border-radius: 5px; font-size: 14px; text-align: center; }</style>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleGPS() {
  String json = "{";
  json += "\"lat\":" + String(latitude, 6) + ",";
  json += "\"lng\":" + String(longitude, 6) + ",";
  json += "\"speed\":" + String(speed_kmh, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", handleRoot);
  server.on("/gps", handleGPS);
  server.begin();
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    speed_kmh = gps.speed.kmph();
  }
  server.handleClient();
}