import time
import math
import alarm
import board
import digitalio
import esp32
import busio
import displayio
import terminalio
from adafruit_ssd1681 import SSD1681
import adafruit_gps
import adafruit_icm20x
import tinys3
import wifi
import ssl
import socketpool
import adafruit_requests
import adafruit_ntp
import json
import rtc
from adafruit_display_text import label

# Import the date data from dates.py
from dates import date_coordinates

# Path to your YAML config file
config_file_path = "/config.yaml"

# Setup for E-Ink Display
displayio.release_displays()

spi = board.SPI()  # Uses SCK and MOSI
epd_cs = board.D34
epd_dc = board.D7
epd_reset = board.D43
epd_busy = board.D44
display_bus = fourwire.FourWire(
    spi, command=epd_dc, chip_select=epd_cs, reset=epd_reset, baudrate=1000000
)

# Initialize the e-ink display with 200x200 pixels resolution
display = SSD1681(display_bus, width=200, height=200, busy_pin=epd_busy) # rotation=180

# Setup for GPS (UART)
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)

# Turn on the basic GGA and RMC info
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')  # 1 Hz update rate

# Setup for IMU (I2C)
# i2c = busio.I2C(board.SCL, board.SDA)
# icm = adafruit_icm20x.ICM20948(i2c)

# Magnetic declination for your location (in degrees)
declination_angle = 2.12  # You can adjust this value based on your location

# Earth radius in kilometers
EARTH_RADIUS_KM = 6371.0

# Function to parse YAML-like config file
def load_config(file_path):
    config = {}
    with open(file_path, "r") as file:
        for line in file:
            line = line.strip()  # Remove any leading/trailing whitespace
            if line and ": " in line:  # Ensure the line has the expected "key: value" format
                try:
                    key, value = line.split(": ", 1)  # Split on the first ": " only
                    config[key.strip()] = value.strip().strip('"')
                except ValueError as e:
                    print(f"Skipping malformed line: {line}")
            elif line:  # If the line exists but isn't valid, warn the user
                print(f"Warning: Skipping line (not key-value format): {line}")
    return config

# Function to get Wi-Fi access point data (MAC addresses and signal strengths)
def get_wifi_data():
    wifi_data = []
    for network in wifi.radio.start_scanning_networks():
        wifi_data.append({
            "macAddress": network.bssid.hex(":").upper(),
            "signalStrength": network.rssi
        })
    wifi.radio.stop_scanning_networks()
    return wifi_data

# Function to get time zone offset using Google's Time Zone API
def get_time_zone(latitude, longitude, google_api_key):
    pool = socketpool.SocketPool(wifi.radio)
    requests = adafruit_requests.Session(pool, ssl.create_default_context())
    
    # Google Time Zone API endpoint
    google_time_zone_url = f"https://maps.googleapis.com/maps/api/timezone/json?location={latitude},{longitude}&timestamp={int(time.time())}&key={google_api_key}"
    
    try:
        # Send GET request to the Time Zone API
        response = requests.get(google_time_zone_url)
        
        # Parse the JSON response
        if response.status_code == 200:
            time_zone_data = response.json()
            time_offset = time_zone_data["rawOffset"]  # Base time offset from UTC (in seconds)
            dst_offset = time_zone_data.get("dstOffset", 0)  # Daylight savings offset (in seconds)
            total_offset = (time_offset + dst_offset) // 3600  # Convert to hours
            
            print(f"Time Zone: {time_zone_data['timeZoneId']}, Offset: {total_offset} hours from UTC")
            return total_offset
        else:
            print(f"Failed to get time zone. Status code: {response.status_code}")
            return 0
    except Exception as e:
        print(f"Error fetching time zone: {e}")
        return 0

# Function to fetch current time from an NTP server and adjust with the dynamic time zone
def fetch_ntp_time(tz_offset_hours):
    pool = socketpool.SocketPool(wifi.radio)
    ntp = adafruit_ntp.NTP(pool, tz_offset=tz_offset_hours)
    try:
        current_time = ntp.datetime
        rtc.RTC().datetime = current_time  # Set the internal clock
        print("Time synced to NTP:", current_time)
        return current_time
    except Exception as e:
        print(f"Error fetching time from NTP: {e}")
        return None

# Function to get geolocation using Google's Geolocation API
def get_location_time(config_file_path):
    
    # Load the configuration
    config = load_config(config_file_path)
    wifi_ssid = config["wifi_ssid"]
    wifi_password = config["wifi_password"]
    google_api_key = config["google_api_key"]

    # Turn Wi-Fi on and connect
    print("Turning on Wi-Fi...")
    wifi.radio.enabled = True  # Ensure Wi-Fi is enabled
    print(f"Connecting to {wifi_ssid}...")
    try:
        wifi.radio.connect(wifi_ssid, wifi_password)
        print(f"Connected to {wifi_ssid}")
    except Exception as e:
        print(f"Failed to connect to Wi-Fi: {e}")
        return None

    # Wi-Fi data (list of dictionaries with MAC and signal strength)
    wifi_data = get_wifi_data()

    # Prepare payload for Google Geolocation API
    geo_payload = {
        "wifiAccessPoints": wifi_data
    }

    # Create HTTP request
    pool = socketpool.SocketPool(wifi.radio)
    requests = adafruit_requests.Session(pool, ssl.create_default_context())

    google_api_url = f"https://www.googleapis.com/geolocation/v1/geolocate?key={google_api_key}"

    try:
        # Send POST request to Google's API
        response = requests.post(google_api_url, json=geo_payload)
        
        # Check for a successful response
        if response.status_code == 200:
            location_data = response.json()
            latitude = location_data['location']['lat']
            longitude = location_data['location']['lng']
            accuracy = location_data['accuracy']
            print(f"Latitude: {latitude}, Longitude: {longitude}, Accuracy: {accuracy} meters")

            # Fetch time zone using the latitude and longitude TODO figure out DST
            tz_offset_hours = get_time_zone(latitude, longitude, google_api_key)

            # Fetch the NTP time with the correct time zone offset
            current_time = fetch_ntp_time(tz_offset_hours)

            return latitude, longitude, accuracy, current_time
        else:
            print(f"Error {response.status_code}: {response.text}")
            return None

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

    # Turn Wi-Fi off to save power
    print("Turning off Wi-Fi to save power...")
    wifi.radio.enabled = False  # Disable Wi-Fi

    return None


# Function to calculate the day of the year
def day_of_year(month, day):
    days_in_months = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334]
    return days_in_months[month - 1] + day

# Function to find the closest date based on the current GPS date
def find_closest_date(gps_month, gps_day):
    current_day_of_year = day_of_year(gps_month, gps_day)
    closest_diff = 999999  # Large number for comparison
    closest_date = None

    for date in date_coordinates:
        date_day_of_year = day_of_year(date["month"], date["day"])
        
        # Calculate the difference, accounting for year wrapping
        diff = date_day_of_year - current_day_of_year
        if diff < 0:
            diff += 365  # If the date is in the next year, wrap around

        # If this date is closer than the previously found closest date
        if diff < closest_diff:
            closest_diff = diff
            closest_date = date

    return closest_date

# Function to calculate heading from GPS coordinates to target coordinates
def calculate_heading(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    heading = math.atan2(y, x)
    
    heading = math.degrees(heading)
    if heading < 0:
        heading += 360
    return heading

# Function to calculate the distance between two coordinates using the Haversine formula
def calculate_distance(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) * math.sin(dlon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return EARTH_RADIUS_KM * c

# Function to get compass orientation from magnetometer readings
def get_compass_orientation(declination):
    mag = icm.magnetic  # Get magnetometer data
    heading = math.atan2(mag[1], mag[0])  # atan2(mag_y, mag_x)
    heading = math.degrees(heading) + declination  # Adjust for magnetic declination
    
    if heading < 0:
        heading += 360
    elif heading >= 360:
        heading -= 360

    return heading

# Function to update display with date and distance
def update_display(day, month, year, distance):
    display_group = displayio.Group()  # Create a group to hold display content

    # Text for the date
    date_text = f"{day}/{month}/{year}"
    
    # Text for the distance
    distance_text = f"{distance:.2f} km"

    # Add both date and distance to the display group
    text_area = label.Label(terminalio.FONT, text=date_text, color=0x000000)
    text_area.x = 20
    text_area.y = 50
    display_group.append(text_area)

    text_area2 = label.Label(terminalio.FONT, text=distance_text, color=0x000000)
    text_area2.x = 20
    text_area2.y = 80
    display_group.append(text_area2)

    print("About to sleep for refresh...")

    # Add a delay to avoid refreshing too soon
    time.sleep(display.time_to_refresh)  # Wait for the display to be ready for another refresh

    print("I've slept...")

    display.root_group = display_group  # Set the root group for the display
    display.refresh()  # Refresh the display to show the new content

# Function to enter deep sleep setup
def enter_deep_sleep(duration_seconds):
    
    print(f"Entering deep sleep for {duration_seconds} seconds...")

    # Set a timer alarm to wake up after the specified number of seconds
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + duration_seconds)

    # Enter deep sleep until the alarm goes off
    alarm.exit_and_deep_sleep_until_alarms(time_alarm)

def main():
    
    print("Good morning!")

    epd_ena.value = True # Switch on display

    # Example usage
    coordinates = get_location_time(config_file_path)

    if coordinates:
        latitude, longitude, accuracy, current_time = coordinates
        print(f"Coordinates: {latitude}, {longitude}, with {accuracy} meters accuracy.")
        print(f"Current time: {current_time}")
    else:
        print("Could not determine location.")
        update_display(1, 1, 1111, -1)

    time.sleep(2)
    print("I'm going to try updating the display")
    time.sleep(2)
    update_display(1, 1, 2024, 3141)
    print("I've updated display")

    # Find the closest date
    closest_date = find_closest_date(current_time.tm_mon, current_time.tm_mday)

    # Calculate the distance and heading
    distance = calculate_distance(latitude, longitude, closest_date["latitude"], closest_date["longitude"])
    heading = calculate_heading(latitude, longitude, closest_date["latitude"], closest_date["longitude"])

    print(f"Target Heading: {heading:.2f} degrees")
    print(f"Target Distance: {distance:.2f} km")

    # Update the e-ink display with the closest date and distance
    update_display(closest_date["day"], closest_date["month"], closest_date["year"], distance)

    # Check battery voltage
    voltage = get_battery_voltage()
    print(f"Battery Voltage: {voltage:.2f} V")

    
    epd_ena.value = False # Switch off display

    # Enter deep sleep for 10 seconds
    enter_deep_sleep(3600)

while True:
    main()