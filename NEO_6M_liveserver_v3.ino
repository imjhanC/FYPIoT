#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define RX 14
#define TX 12
#define GPS_BAUD 9600
#define HDOP_THRESHOLD 2.0    // Maximum acceptable HDOP value
#define MOVING_AVERAGE_SIZE 5  // Size of moving average window
#define MIN_SPEED_THRESHOLD 2.0  // Minimum speed to consider movement (km/h)

const char *ssid = "ria ria 327_2.4GHZ";
const char *password = "";

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX, TX);
ESP8266WebServer server(80);

// Structs for storing filtered data
struct LocationData {
    double lat;
    double lng;
    double speed;
};

// Circular buffer for moving average
LocationData locationBuffer[MOVING_AVERAGE_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

double latitude = 0.0, longitude = 0.0, speed_kmh = 0.0;
int minSatellites = 6;  // Increased minimum satellites
bool gpsAvailable = false;

// Calculate moving average
LocationData calculateMovingAverage() {
    LocationData avg = {0, 0, 0};
    int count = bufferFilled ? MOVING_AVERAGE_SIZE : bufferIndex;
    
    if (count == 0) return avg;
    
    for (int i = 0; i < count; i++) {
        avg.lat += locationBuffer[i].lat;
        avg.lng += locationBuffer[i].lng;
        avg.speed += locationBuffer[i].speed;
    }
    
    avg.lat /= count;
    avg.lng /= count;
    avg.speed /= count;
    
    return avg;
}

// Add new reading to buffer
void addToBuffer(double lat, double lng, double speed) {
    locationBuffer[bufferIndex] = {lat, lng, speed};
    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_SIZE;
    if (bufferIndex == 0) bufferFilled = true;
}

// Calculate distance between two points
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // Earth's radius in meters
    double dLat = (lat2 - lat1) * PI / 180;
    double dLon = (lon2 - lon1) * PI / 180;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * PI / 180) * cos(lat2 * PI / 180) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

void handleRoot() {
    String html = "<!DOCTYPE html>";
    html += "<html><head><title>GPS Tracker</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<script src='https://unpkg.com/leaflet@1.7.1/dist/leaflet.js'></script>";
    html += "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.7.1/dist/leaflet.css'/>";
    html += "<style>";
    html += "body { margin: 0; padding: 0; }";
    html += "#map { height: 85vh; width: 100%; }";
    html += ".info-panel { padding: 10px; background: white; box-shadow: 0 0 10px rgba(0,0,0,0.2); }";
    html += ".speed-label { background: white; padding: 5px; border-radius: 5px; box-shadow: 0 0 5px rgba(0,0,0,0.2); }";
    html += "</style></head>";
    html += "<body>";
    html += "<div class='info-panel'>";
    html += "<h2>GPS Tracker</h2>";
    html += "<p id='status'>Waiting for GPS signal...</p>";
    html += "<p id='coordinates'></p>";
    html += "<p id='speed'></p>";
    html += "</div>";
    html += "<div id='map'></div>";
    
    html += "<script>";
    html += "var map, marker, speedMarker, path;";
    html += "var locations = [];";
    
    html += "function initMap() {";
    html += "    map = L.map('map').setView([0, 0], 15);";
    html += "    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {";
    html += "        attribution: '© OpenStreetMap contributors'";
    html += "    }).addTo(map);";
    html += "    path = L.polyline([], {color: 'blue', weight: 3}).addTo(map);";
    html += "    updateLocation();";
    html += "}";
    
    html += "function updateLocation() {";
    html += "    fetch('/gps')";
    html += "        .then(response => response.json())";
    html += "        .then(data => {";
    html += "            if (data.lat !== 0 && data.lng !== 0) {";
    html += "                document.getElementById('status').innerHTML = '<strong>GPS Signal: Active</strong>';";
    html += "                document.getElementById('coordinates').innerHTML = 'Location: ' + data.lat.toFixed(6) + ', ' + data.lng.toFixed(6);";
    html += "                document.getElementById('speed').innerHTML = 'Speed: ' + data.speed.toFixed(1) + ' km/h';";
    
    html += "                var pos = [data.lat, data.lng];";
    html += "                locations.push(pos);";
    
    html += "                if (!marker) {";
    html += "                    marker = L.marker(pos).addTo(map);";
    html += "                    map.setView(pos, 15);";
    html += "                } else {";
    html += "                    marker.setLatLng(pos);";
    html += "                    map.panTo(pos);";
    html += "                }";
    
    html += "                if (!speedMarker) {";
    html += "                    speedMarker = L.marker(pos, {";
    html += "                        icon: L.divIcon({";
    html += "                            className: 'speed-label',";
    html += "                            html: '<b>' + data.speed.toFixed(1) + ' km/h</b>',";
    html += "                            iconSize: [80, 20]";
    html += "                        })";
    html += "                    }).addTo(map);";
    html += "                } else {";
    html += "                    speedMarker.setLatLng(pos);";
    html += "                    speedMarker.setIcon(L.divIcon({";
    html += "                        className: 'speed-label',";
    html += "                        html: '<b>' + data.speed.toFixed(1) + ' km/h</b>',";
    html += "                        iconSize: [80, 20]";
    html += "                    }));";
    html += "                }";
    
    html += "                path.setLatLngs(locations);";
    html += "            } else {";
    html += "                document.getElementById('status').innerHTML = '<strong>Searching for GPS signal...</strong>';";
    html += "            }";
    html += "        })";
    html += "        .catch(err => {";
    html += "            document.getElementById('status').innerHTML = '<strong>Error: </strong>' + err.message;";
    html += "        });";
    html += "    setTimeout(updateLocation, 2000);";
    html += "}";
    
    html += "window.onload = initMap;";
    html += "</script></body></html>";
    
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
    
    // Set GPS module to airborne mode for better accuracy
    gpsSerial.begin(GPS_BAUD);
    // Configure update rate and navigation mode
    // NEO-6M configuration commands (UBX protocol)
    uint8_t setNavMode[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 
        0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
    };
    gpsSerial.write(setNavMode, sizeof(setNavMode));
    
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
    static unsigned long lastValidUpdate = 0;
    static LocationData lastValidLocation = {0, 0, 0};
    
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    if (gps.location.isUpdated() && 
        gps.satellites.value() >= minSatellites && 
        gps.hdop.hdop() <= HDOP_THRESHOLD) {
        
        double newLat = gps.location.lat();
        double newLng = gps.location.lng();
        double newSpeed = gps.speed.kmph();
        
        // Filter out erroneous speed readings when stationary
        if (newSpeed < MIN_SPEED_THRESHOLD) {
            newSpeed = 0;
        }
        
        // Check if movement is realistic
        if (lastValidUpdate > 0) {
            double timeDelta = (millis() - lastValidUpdate) / 1000.0; // seconds
            double distance = calculateDistance(lastValidLocation.lat, lastValidLocation.lng, newLat, newLng);
            double calculatedSpeed = (distance / timeDelta) * 3.6; // Convert to km/h
            
            // If calculated speed is unrealistic, ignore this reading
            if (calculatedSpeed > 200) { // Max realistic speed threshold
                return;
            }
        }
        
        // Add to moving average buffer
        addToBuffer(newLat, newLng, newSpeed);
        
        // Calculate filtered values
        LocationData avgLocation = calculateMovingAverage();
        latitude = avgLocation.lat;
        longitude = avgLocation.lng;
        speed_kmh = avgLocation.speed;
        
        lastValidLocation = {newLat, newLng, newSpeed};
        lastValidUpdate = millis();
        gpsAvailable = true;
    }
    
    server.handleClient();
}