// Libraries
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "Adafruit_ThinkInk.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "dates.h"  // Header file with date data

//// Pinout definitions

// E-ink display
#define EPD_DC 6
#define EPD_CS 10
#define EPD_BUSY 8  // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS -1
#define EPD_RESET 7   // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI  // primary SPI
#define ENA 9
// 1.54" Monochrome displays with 200x200 pixels and SSD1681 chipset
ThinkInk_154_Mono_D67 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// GPS (TX followed by RX)
SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);

// IMU (I2C)
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

//// Constants and global variables

#define DEG_TO_RAD 0.017453292519943295  // π / 180
#define RAD_TO_DEG 57.29577951308232     // 180 / π

#define DEBUG 1
volatile int wakeUpLimit = 10800;
volatile int wakeUpCount = wakeUpLimit;
int old_year = 0, old_month = 0, old_day = 0;  // Last chosen year/month/day
int new_year, new_month, new_day;              // New calculated year/month//day
float latitude, longitude;                     // Variables to store coordinates
float target_heading, current_heading;

// Set up the watchdog timer for 8-second intervals
void setupWatchdogTimer() {
  cli();        // Disable interrupts
  wdt_reset();  // Reset the watchdog timer

  // Set watchdog timer to 8-second interval
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDP3) | (1 << WDP0);  // 8-second timeout
  WDTCSR |= (1 << WDIE);               // Enable watchdog interrupt
  sei();                               // Enable interrupts
}

// Enter power-save mode and retain SRAM
void enterPowerSaveMode() {
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);  // Set sleep mode to Power-Save
  sleep_enable();                       // Enable sleep mode
  sleep_mode();                         // Enter sleep mode

  // The program continues here after waking up from sleep
  sleep_disable();  // Disable sleep mode after waking up
}

// Watchdog timer interrupt service routine
ISR(WDT_vect) {
  // Arduino wakes up here after the watchdog timer expires
}

// Function to calculate day-of-year from month and day
int dayOfYear(int month, int day) {
  const int daysInMonths[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
  return daysInMonths[month - 1] + day;
}

// Function to find the closest month and day, ignoring the year, and return the coordinates
void findClosestDate(int gpsMonth, int gpsDay, int* month, int* day, float* latitude, float* longitude) {

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

// Check compass heading

// Set compass to heading

void setup() {

  // Initialize serial
  if (DEBUG) {
    Serial.begin(9600);
    Serial.println("Watchdog timer set...");
    while (!Serial) {
      delay(10);
    }
  }

  // Set up the watchdog timer for 8-second intervals
  setupWatchdogTimer();
  if (DEBUG) {
    Serial.println("Watchdog timer set...");
  }

  // Initialize display
  display.begin(THINKINK_MONO);
  if (DEBUG) {
    Serial.println("Display initialized...");
  }

  // Initialize GPS
  GPS.begin(9600);

  // Initialize IMU
}

void loop() {

  // Check if we need to go back to sleep
  if (wakeUpCount < wakeUpLimit) {
    enterPowerSaveMode();
    wakeUpCount++;
    if (DEBUG) {
      Serial.print("Woke up count: ");
      Serial.println(wakeUpCount);
    }
  } else {

    // Good morning!
    if (DEBUG) {
      Serial.println("Good morning!");
    }

    // Check GPS
    gpsSerial.write(0xFF);  // Wake up GPS
    delay(100);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // Set to send minimum data only
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      // 1 Hz update rate
    delay(100);
    while (!GPS.fix) {                              // keep checking until you get a GPS fix
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
          return;                        // we can fail to parse a sentence in which case we should just wait for another
      }
    }
    if (DEBUG) {
      Serial.println("GPS fix found!");
    }
    delay(100);
    GPS.sendCommand("$PMTK225,4*2F");  // Put GPS to back up mode

    // Find closest date
    findClosestDate(GPS.month, GPS.day, &new_month, &new_day, &latitude, &longitude);

    // Update display if different
    if (new_month != old_month && new_day != old_day) {
      display.clearBuffer();
      display.setTextSize(2);
      display.setCursor((display.width() - 180) / 2, (display.height() - 24) / 2);
      delay(20);
      display.setTextColor(EPD_BLACK);
      delay(20);
      display.print(String(new_day) + " / " + String(new_month));
      display.display();
      old_month = new_month;
      old_day = new_day;
    }

    // Find heading
    target_heading = calculateHeading((GPS.latitude, 4), (GPS.longitude, 4), latitude, longitude);
    if (DEBUG) {
      Serial.print("Target heading: ");
      Serial.println(target_heading);
    }

    // Check IMU

    // Update needle position if different and 5V not present

    // Work out how long to sleep for

    // Go to sleep
    enterPowerSaveMode();
  }
}
