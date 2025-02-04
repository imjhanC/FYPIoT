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

double latitude = 0.0, longitude = 0.0, speed_kmh = 0.0;
int minSatellites = 4;
bool gpsAvailable = false;  // Track if valid GPS data is available

void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html><head><title>GPS Tracker</title>";
  html += "<script src='https://unpkg.com/leaflet/dist/leaflet.js'></script>";
  html += "<link rel='stylesheet' href='https://unpkg.com/leaflet/dist/leaflet.css'/>";
  html += "<style>#map { height: 100vh; }</style></head><body>";
  html += "<h2>GPS Tracker</h2>";
  html += "<p id='status'>Waiting for GPS signal...</p>";
  html += "<div id='map'></div>";
  html += "<script>var map, marker, speedMarker;";
  html += "function initMap() {";
  html += "  fetch('/gps').then(response => response.json()).then(data => {";
  html += "    if (data.lat !== 0 && data.lng !== 0) {";
  html += "      document.getElementById('status').innerText = 'GPS Data Received';";
  html += "      map = L.map('map').setView([data.lat, data.lng], 15);";
  html += "      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);";
  html += "      marker = L.marker([data.lat, data.lng]).addTo(map);";
  html += "      speedMarker = L.marker([data.lat, data.lng], { icon: L.divIcon({ className: 'speed-label', html: '<b>Speed: ' + data.speed + ' km/h</b>', iconSize: [100, 40] }) }).addTo(map);";
  html += "      setInterval(updateLocation, 2000);";
  html += "    } else { setTimeout(initMap, 2000); }";
  html += "  }).catch(err => setTimeout(initMap, 2000));";
  html += "}";
  html += "function updateLocation() {";
  html += "  fetch('/gps').then(response => response.json()).then(data => {";
  html += "    marker.setLatLng([data.lat, data.lng]); map.setView([data.lat, data.lng]);";
  html += "    speedMarker.setLatLng([data.lat, data.lng]);";
  html += "    speedMarker.setIcon(L.divIcon({ className: 'speed-label', html: '<b>Speed: ' + data.speed + ' km/h</b>', iconSize: [100, 40] }));";
  html += "  });";
  html += "}";
  html += "initMap();";
  html += "</script>";
  html += "<style>.speed-label { background: white; padding: 5px; border-radius: 5px; font-size: 14px; text-align: center; }</style>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleGPS() {
  if (!gpsAvailable) {
    server.send(200, "application/json", "{\"lat\":0,\"lng\":0,\"speed\":0}");
    return;
  }

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
  Serial.println("\nConnected!");
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

  if (gps.location.isUpdated() && gps.satellites.value() >= minSatellites) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    speed_kmh = gps.speed.kmph();
    gpsAvailable = true;
  }

  server.handleClient();
}

