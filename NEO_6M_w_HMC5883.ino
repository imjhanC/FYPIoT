#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define RX 14  // D5 on ESP8266
#define TX 12  // D6 on ESP8266
#define GPS_BAUD 9600
#define HDOP_THRESHOLD 2.0    
#define MOVING_AVERAGE_SIZE 5  
#define MIN_SPEED_THRESHOLD 2.0

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of Software Serial
SoftwareSerial gpsSerial(RX, TX);

// Create an instance of the HMC5883L
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

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
int minSatellites = 6;
bool gpsAvailable = false;

// Previous functions remain the same
LocationData calculateMovingAverage() {
    // ... (keep existing function as is)
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

void addToBuffer(double lat, double lng, double speed) {
    // ... (keep existing function as is)
    locationBuffer[bufferIndex] = {lat, lng, speed};
    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_SIZE;
    if (bufferIndex == 0) bufferFilled = true;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // ... (keep existing function as is)
    const double R = 6371000;
    double dLat = (lat2 - lat1) * PI / 180;
    double dLon = (lon2 - lon1) * PI / 180;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * PI / 180) * cos(lat2 * PI / 180) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

// Get compass direction string from heading
String getDirectionFromHeading(float heading) {
    if (heading >= 337.5 || heading < 22.5) return "North";
    if (heading >= 22.5 && heading < 67.5) return "Northeast";
    if (heading >= 67.5 && heading < 112.5) return "East";
    if (heading >= 112.5 && heading < 157.5) return "Southeast";
    if (heading >= 157.5 && heading < 202.5) return "South";
    if (heading >= 202.5 && heading < 247.5) return "Southwest";
    if (heading >= 247.5 && heading < 292.5) return "West";
    if (heading >= 292.5 && heading < 337.5) return "Northwest";
    return "Unknown";
}

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin();
    
    // Initialize GPS
    gpsSerial.begin(GPS_BAUD);
    Serial.println("Software Serial started at 9600 baud rate");
    
    // Initialize HMC5883L
    if (!mag.begin()) {
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        while (1);
    }
    Serial.println("HMC5883L Initialized");
    
    // Set GPS module to airborne mode
    uint8_t setNavMode[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 
        0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
    };
    gpsSerial.write(setNavMode, sizeof(setNavMode));
}

void loop() {
    static unsigned long lastValidUpdate = 0;
    static LocationData lastValidLocation = {0, 0, 0};
    static unsigned long start = millis();
    
    // Get compass reading
    sensors_event_t event;
    mag.getEvent(&event);
    
    // Calculate heading
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    heading = heading * 180 / PI;  // Convert to degrees
    if (heading < 0) heading += 360;  // Ensure heading is positive
    String direction = getDirectionFromHeading(heading);
    
    while (millis() - start < 1000) {
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
                double timeDelta = (millis() - lastValidUpdate) / 1000.0;
                double distance = calculateDistance(lastValidLocation.lat, lastValidLocation.lng, newLat, newLng);
                double calculatedSpeed = (distance / timeDelta) * 3.6;
                
                if (calculatedSpeed > 200) {
                    continue;
                }
            }
            
            addToBuffer(newLat, newLng, newSpeed);
            
            LocationData avgLocation = calculateMovingAverage();
            latitude = avgLocation.lat;
            longitude = avgLocation.lng;
            speed_kmh = avgLocation.speed;
            
            lastValidLocation = {newLat, newLng, newSpeed};
            lastValidUpdate = millis();
            gpsAvailable = true;
            
            // Print all data including compass heading
            Serial.println("\n--- GPS and Compass Update ---");
            Serial.print("LAT: ");
            Serial.println(latitude, 6);
            Serial.print("LONG: "); 
            Serial.println(longitude, 6);
            Serial.print("SPEED (km/h) = "); 
            Serial.println(speed_kmh, 2);
            Serial.print("Compass Heading: ");
            Serial.print(heading, 1);
            Serial.print("° - Direction: ");
            Serial.println(direction);
            Serial.print("ALT (m) = "); 
            Serial.println(gps.altitude.meters());
            Serial.print("ALT (ft) = "); 
            Serial.println(gps.altitude.feet());
            Serial.print("HDOP = "); 
            Serial.println(gps.hdop.value() / 100.0);
            Serial.print("Satellites = "); 
            Serial.println(gps.satellites.value());
            
            if (gps.course.isUpdated()) {
                Serial.print("GPS Course (degrees) = "); 
                Serial.println(gps.course.deg());
            }
            
            Serial.print("GPS Fix: ");
            Serial.println(gps.location.isValid() ? "Valid" : "Invalid");
            
            Serial.print("Time in UTC: ");
            Serial.println(String(gps.date.year()) + "/" + 
                         String(gps.date.month()) + "/" + 
                         String(gps.date.day()) + "," + 
                         String(gps.time.hour()) + ":" + 
                         String(gps.time.minute()) + ":" + 
                         String(gps.time.second()));
        }
    }
    start = millis();
}