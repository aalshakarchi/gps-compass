// Libraries
#include "Adafruit_ThinkInk.h"
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include "dates.h"  // Header file with date data

//// Pinout definitions

// E-ink display
#define EPD_DC 7
#define EPD_CS 34
#define EPD_BUSY 1  // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS -1
#define EPD_RESET 6   // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI  // primary SPI
#define ENA 2
// 1.54" Monochrome displays with 200x200 pixels and SSD1681 chipset
ThinkInk_154_Mono_D67 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// GPS (ESP32 HardwareSerial)
HardwareSerial gpsSerial(1);  // Use UART1 on ESP32
Adafruit_GPS GPS(&gpsSerial);

// IMU (I2C)
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535;  // Delay between measurements for testing

// Pin definition for battery voltage measurement on TinyPICO
#define VBAT_PIN 35

//// Constants and global variables

#define DEG_TO_RAD 0.017453292519943295  // π / 180
#define RAD_TO_DEG 57.29577951308232     // 180 / π
#define EARTH_RADIUS_KM 6371.0           // Radius of the Earth in kilometers

#define DEBUG 1
int old_year = 0, old_month = 0, old_day = 0;  // Last chosen year/month/day
int new_year, new_month, new_day;              // New calculated year/month//day
float latitude, longitude;                     // Variables to store coordinates
float target_heading, current_heading, old_orientation, new_orientation, distance;

// Define magnetic declination for your location (in degrees)
// Example: 13° East of true North, so declination = 13.0
float declinationAngle = 2.12;  // You can adjust this value based on your location

// ESP32 deep sleep setup
void setupDeepSleepTimer(int seconds) {
  // Set the ESP32 to wake up after 8 seconds
  esp_sleep_enable_timer_wakeup(seconds * 1000000);  // 8 seconds in microseconds
}

void enterDeepSleepMode(int seconds) {
  if (DEBUG) {
    Serial.println("Entering deep sleep...");
  }
  setupDeepSleepTimer(seconds);
  esp_deep_sleep_start();  // Enter deep sleep mode
}

// Function to calculate day-of-year from month and day
int dayOfYear(int month, int day) {
  const int daysInMonths[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
  return daysInMonths[month - 1] + day;
}

// Function to find the closest month and day, ignoring the year, and return the coordinates
void findClosestDate(int gpsMonth, int gpsDay, int* year, int* month, int* day, float* latitude, float* longitude) {

  int currentDayOfYear = dayOfYear(gpsMonth, gpsDay);  // Calculate current day-of-year
  DateCoordinate closestDate;
  int closestDiff = 999999;  // Large number for comparison

  // Loop through all the dates and find the closest one based on month and day
  for (int i = 0; i < numDates; i++) {
    DateCoordinate dateCoord = dateCoordinates[i];

    // Calculate the day-of-year for the date
    int dateDayOfYear = dayOfYear(dateCoord.month, dateCoord.day);

    // Calculate the difference, accounting for year wrapping
    int diff = dateDayOfYear - currentDayOfYear;

    if (diff < 0) {
      diff += 365;  // If the date is in the next year, wrap around
    }

    // If this date is closer than the previously found closest date
    if (diff < closestDiff) {
      closestDiff = diff;
      closestDate = dateCoord;
    }
  }

  // Set the month, day, latitude, and longitude using pointers
  *year = closestDate.year;
  *month = closestDate.month;
  *day = closestDate.day;
  *latitude = closestDate.latitude;
  *longitude = closestDate.longitude;
}

// Function to calculate heading from GPS coordinates (gps_lat, gps_lon) to target coordinates (target_lat, target_lon)
float calculateHeading(float gps_lat, float gps_lon, float target_lat, float target_lon) {
  // Convert degrees to radians
  gps_lat = gps_lat * DEG_TO_RAD;
  gps_lon = gps_lon * DEG_TO_RAD;
  target_lat = target_lat * DEG_TO_RAD;
  target_lon = target_lon * DEG_TO_RAD;

  // Calculate the difference in longitudes
  float dLon = target_lon - gps_lon;

  // Compute the heading
  float y = sin(dLon) * cos(target_lat);
  float x = cos(gps_lat) * sin(target_lat) - sin(gps_lat) * cos(target_lat) * cos(dLon);
  float heading = atan2(y, x);

  // Convert radians to degrees
  heading = heading * RAD_TO_DEG;

  // Normalize the result to 0-360 degrees
  if (heading < 0) {
    heading += 360;
  }

  return heading;
}

// Function to calculate distance between two coordinates using the Haversine formula
float calculateDistance(float gps_lat, float gps_lon, float target_lat, float target_lon) {
  // Convert degrees to radians
  gps_lat = gps_lat * DEG_TO_RAD;
  gps_lon = gps_lon * DEG_TO_RAD;
  target_lat = target_lat * DEG_TO_RAD;
  target_lon = target_lon * DEG_TO_RAD;

  // Haversine formula to calculate the distance
  float dLat = target_lat - gps_lat;
  float dLon = target_lon - gps_lon;

  float a = sin(dLat / 2) * sin(dLat / 2) + cos(gps_lat) * cos(target_lat) * sin(dLon / 2) * sin(dLon / 2);

  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // Distance in kilometers
  float distance = EARTH_RADIUS_KM * c;

  return distance;
}

// Function to calculate compass orientation from magnetometer readings
float calculateOrientation(float declination) {

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  // Get sensor data from ICM20948
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Display magnetometer data
  if (DEBUG) {
    Serial.print("Mag X: ");
    Serial.print(mag.magnetic.x);
    Serial.print(" \tMag Y: ");
    Serial.print(mag.magnetic.y);
    Serial.print(" \tMag Z: ");
    Serial.println(mag.magnetic.z);
  }

  // Calculate heading using atan2
  float orientation = atan2(mag.magnetic.y, mag.magnetic.x);

  // Print the calculated heading
  if (DEBUG) {
    Serial.print("Compass orientation: ");
    Serial.print(orientation);
    Serial.println(" degrees");
  }

  // Convert radians to degrees
  orientation = orientation * RAD_TO_DEG;

  // Adjust for declination
  orientation += orientation;

  // Normalize heading to 0 - 360 degrees
  if (orientation < 0) {
    orientation += 360;
  } else if (orientation >= 360) {
    orientation -= 360;
  }

  // Somethink to work out offset of motor to IMU north?

  return orientation;
}

// Function to measure battery voltage
float readBatteryVoltage() {
  int raw = analogRead(VBAT_PIN);                 // Read the raw ADC value
  float voltage = (float)raw / 4095.0 * 3.3 * 2;  // Scale ADC value to actual battery voltage (factor of 2 for voltage divider)
  return voltage;
}

void setup() {

  // Initialize serial
  if (DEBUG) {
    Serial.begin(115200);
    Serial.println("Starting up...");
  }

  // Disable WiFi and Bluetooth to save power
  WiFi.disconnect(true);  // Disconnect from any network
  WiFi.mode(WIFI_OFF);    // Turn off Wi-Fi
  delay(100);             // Give some time for the Wi-Fi to shut down
  // esp_bluedroid_disable();      // Disable Bluetooth Classic (if using it)
  // esp_bluedroid_deinit();       // De-initialize Bluetooth Classic stack
  esp_bt_controller_disable();  // Disable the Bluetooth controller

  // Initialize display
  display.begin(THINKINK_MONO);
  if (DEBUG) {
    Serial.println("Display initialized...");
  }

  // Initialize GPS (UART1 on ESP32)
  gpsSerial.begin(9600, SERIAL_8N1, 44, 43);  // Example RX, TX pins for GPS
  GPS.begin(9600);

  // Initialize IMU
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");

  // ADC setup (ESP32 ADC1 - 12-bit resolution)
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation (0 to 3.3V range)
}

void loop() {

  // Good morning!
  if (DEBUG) {
    Serial.println("Good morning!");
  }

  // Print battery voltage TODO make this an if statement
  if (DEBUG) {
    float batteryVoltage = readBatteryVoltage();
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println(" V");
  }

  // Check GPS
  gpsSerial.write(0xFF);  // Wake up GPS
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // Set to send minimum data only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      // 1 Hz update rate
  delay(100);
  while (!GPS.fix) {  // Keep checking until you get a GPS fix
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))  // This also sets the newNMEAreceived() flag to false
        return;
    }
  }
  if (DEBUG) {
    Serial.println("GPS fix found!");
  }
  delay(100);
  GPS.sendCommand("$PMTK225,4*2F");  // Put GPS to back up mode

  // Find closest date
  findClosestDate(GPS.month, GPS.day, &new_year, &new_month, &new_day, &latitude, &longitude);

  // Find target heading
  target_heading = calculateHeading(GPS.latitude, GPS.longitude, latitude, longitude);
  if (DEBUG) {
    Serial.print("Target heading: ");
    Serial.println(target_heading);
  }

  // Find target distance
  distance = calculateDistance(GPS.latitude, GPS.longitude, latitude, longitude);
  if (DEBUG) {
    Serial.print("Target heading: ");
    Serial.println(target_heading);
  }

  // Update display if different
  if (new_month != old_month && new_day != old_day) {

    // Clear the buffer and prepare to write to the display
    display.clearBuffer();

    // Set text size
    display.setTextSize(2);

    // Set cursor for the first line (Date)
    display.setCursor((display.width() - 180) / 2, (display.height() - 48) / 2);  // Adjust Y to move higher for the first line
    display.setTextColor(EPD_BLACK);

    // Print the date (month/day)
    display.print(String(new_day) + " / " + String(new_month) + " / " + String(new_year));

    // Set cursor for the second line (Distance)
    display.setCursor((display.width() - 180) / 2, (display.height() - 24) / 2 + 24);  // Move down for second line (adjust Y)

    // Print the distance in kilometers
    display.print(String(distance, 2) + "km");

    // Display the content
    display.display();

    // Update old values to avoid redundant updates
    old_year = new_year;
    old_month = new_month;
    old_day = new_day;

  }

  // Find current compass orientation
  new_orientation = calculateOrientation(declinationAngle);

  // Move needle if heading and/or orientation are different TODO and if 5V is not present

  // Work out how long to sleep for

  // Go to sleep
  enterDeepSleepMode(86400);
}