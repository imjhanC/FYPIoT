#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Create an instance of the HMC5883L
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  Serial.begin(115200);
  Serial.println("HMC5883L Magnetometer Test");

  // Initialize the magnetometer
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);
  }

  Serial.println("HMC5883L Initialized");
}

void loop() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate heading (in degrees)
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Convert from radians to degrees
  heading = heading * 180 / PI;

  // Ensure heading is positive
  if (heading < 0) {
    heading += 360;
  }

  // Determine compass direction
  String direction;
  if (heading >= 337.5 || heading < 22.5) {
    direction = "North";
  } else if (heading >= 22.5 && heading < 67.5) {
    direction = "Northeast";
  } else if (heading >= 67.5 && heading < 112.5) {
    direction = "East";
  } else if (heading >= 112.5 && heading < 157.5) {
    direction = "Southeast";
  } else if (heading >= 157.5 && heading < 202.5) {
    direction = "South";
  } else if (heading >= 202.5 && heading < 247.5) {
    direction = "Southwest";
  } else if (heading >= 247.5 && heading < 292.5) {
    direction = "West";
  } else if (heading >= 292.5 && heading < 337.5) {
    direction = "Northwest";
  }

  // Print heading and direction
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("° - Direction: ");
  Serial.println(direction);

  delay(500);
}
