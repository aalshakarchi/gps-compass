import time
import math
import alarm
import board
import digitalio
import analogio
import busio
import displayio
import terminalio
from adafruit_ssd1681 import SSD1681
import adafruit_icm20x
import esp32
import tinys3
import wifi
import ssl
import socketpool
import adafruit_requests
import adafruit_ntp
import json
import rtc
from adafruit_display_text import label
from adafruit_display_shapes.rect import Rect
import fourwire
import pwmio
import pulseio

# Import the date data from dates.py
from dates import date_coordinates

# Path to your YAML config file
config_file_path = "/config.yaml"

# Setup for E-Ink Display
displayio.release_displays()

spi = board.SPI()  # Uses SCK and MOSI
epd_cs = board.D34
epd_dc = board.D7
epd_ena = digitalio.DigitalInOut(board.D6)
epd_ena.direction = digitalio.Direction.OUTPUT
epd_reset = board.D43
epd_busy = board.D44
display_bus = fourwire.FourWire(
    spi, command=epd_dc, chip_select=epd_cs, reset=epd_reset, baudrate=1000000
)

# Initialize the e-ink display with 200x200 pixels resolution
display = SSD1681(display_bus, width=200, height=200, busy_pin=epd_busy, rotation=180)

# Setup for IMU (I2C)
i2c = busio.I2C(board.SCL, board.SDA)
icm = adafruit_icm20x.ICM20948(i2c)

# Servo pins
servo_ena = digitalio.DigitalInOut(board.D1)
servo_pin = board.D2
# feedback_pin = board.D3
feedback_pin = digitalio.DigitalInOut(board.D3)
servo_ena.direction = digitalio.Direction.OUTPUT
pwm = pwmio.PWMOut(servo_pin, frequency=50)
# feedback = pulseio.PulseIn(feedback_pin, maxlen=2, idle_state=True)#, maxlen=2)#, idle_state=False)
feedback_pin.direction = digitalio.Direction.INPUT

# Magnetic declination for your location (in degrees)
declination_angle = 2.12  # You can adjust this value based on your location

# Earth radius in kilometers
EARTH_RADIUS_KM = 6371.0

# Low battery
LOW_BATTERY = 3.0

# Servo settings
CYCLE_DURATION_NS = 1089000  # 1089 ms or 1.089 seconds
DUTY_CYCLE_MIN = 0.08
DUTY_CYCLE_MAX = 0.89
ALPHA = 0.5
servo_pulse_width = 1465
needle_offset = 0

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

# Function to find the closest date (before or after) based on the current GPS date
def find_closest_date(gps_month, gps_day):
    # Helper function to calculate day of the year
    def day_of_year(month, day):
        days_in_months = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334]
        return days_in_months[month - 1] + day

    # Helper function to calculate the difference in days between two dates (ignoring year)
    def day_difference(month1, day1, month2, day2):
        # Convert both dates to day of the year
        day_of_year_1 = day_of_year(month1, day1)
        day_of_year_2 = day_of_year(month2, day2)

        # Calculate the difference in days and handle wrapping around the year
        forward_diff = (day_of_year_2 - day_of_year_1) % 365
        backward_diff = (day_of_year_1 - day_of_year_2) % 365

        # Return the smaller of the forward or backward difference
        return min(forward_diff, backward_diff)

    # Initialize variables to store the closest date
    closest_diff = 999999  # Large number for comparison
    closest_date = None

    # Loop through all available dates
    for date in date_coordinates:
        # Calculate the difference in days between the current date and the event date
        diff = day_difference(gps_month, gps_day, date["month"], date["day"])

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
    
    while not i2c.try_lock():
        pass  # Wait until the lock is acquired
    try:
        i2c.writeto(0x69, bytes([0x06, 0x01]))  # Wake IMU up
    finally:
        i2c.unlock()
    time.sleep(0.1)
    
    print("'I've woken up")
    time.sleep(5)

    filtered_heading = 0
    counter = 0
    while counter < 100:

        mag = icm.magnetic  # Get magnetometer data
        heading = math.atan2(-mag[0], -mag[1])  # atan2(mag_y, mag_x), note Y and X are swapped based on how I've installed these plus -
        heading = math.degrees(heading) + declination  # Adjust for magnetic declination
        
        if heading < 0:
            heading += 360
        elif heading >= 360:
            heading -= 360

        filtered_heading = low_pass_filter(filtered_heading, heading, ALPHA)
        # print(f"Filtered heading: {filtered_heading:.0f}, raw heading: {heading}, magY: {mag[1]}, magX: {mag[0]}, counter: {counter}")
        counter += 1
        time.sleep(0.01)

    while not i2c.try_lock():
        pass  # Wait until the lock is acquired
    try:
        i2c.writeto(0x69, bytes([0x06, 0x40])) # Put IMU to sleep
    finally:
        i2c.unlock()
    time.sleep(0.1)

    return heading

# Function to apply low-pass filtering (exponential moving average)
def low_pass_filter(previous_value, new_value, alpha):
    return previous_value * (1 - alpha) + new_value * alpha

# Function to start and stop motor
def motor(state):
    if state:
        duty_cycle = int(servo_pulse_width / 20000 * 65535)
    else:
        duty_cycle = int(1500 / 20000 * 65535) # Stop
    pwm.duty_cycle = duty_cycle

# Function to read the servo position using PWM feedback
def read_servo_position(raw=False):

    filtered_position = 0
    counter = 0

    while counter < 100:

        while not feedback_pin.value:
            pass
        
        # Capture the start time for the high pulse
        start_time = time.monotonic_ns()
        
        # Wait for the pin to go low (end of high pulse)
        while feedback_pin.value:
            pass
        
        # Calculate elapsed time for the high pulse in nanoseconds
        high_pulse_duration = (time.monotonic_ns() - start_time)

        # Wait for the pin to go low (start of low pulse)
        while feedback_pin.value:
            pass
        
        # Capture the start time for the low pulse
        start_time = time.monotonic_ns()
        
        # Wait for the pin to go high (end of low pulse)
        while not feedback_pin.value:
            pass
        
        # Calculate elapsed time for the low pulse in nanoseconds
        low_pulse_duration = (time.monotonic_ns() - start_time)

        if high_pulse_duration > low_pulse_duration:
            duty_cycle = (high_pulse_duration / CYCLE_DURATION_NS)
        else:
            duty_cycle = (1 - low_pulse_duration / CYCLE_DURATION_NS)
        
        position = 360 * (duty_cycle - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)
        
        # Limit position to 0-360 degrees
        if position < 0:
            position = 0
        elif position > 360:
            position = 360

        if raw:
            return position

        filtered_position = low_pass_filter(filtered_position, position, ALPHA)
        counter += 1

    return filtered_position

# Function to control servo movement with Proportional Control
def set_servo_position(target_position):
    Kp = 0.01  # Proportional gain (adjust as needed)
    
    counter = 0
    filtered_position = 0

    # Run servo position for 100 counts until the position stabilizes, before moving servo
    while counter < 100:
        filtered_position = low_pass_filter(filtered_position, read_servo_position(raw=True), ALPHA)
        counter += 1

    # Check if needle is more than 10 degrees further than it needs to be
    if abs(target_position - filtered_position) > 10:

        # Start motor
        motor(True)

        counter = 0

        while True:

            filtered_position = low_pass_filter(filtered_position, read_servo_position(raw=True), ALPHA)
            counter += 1
            
            if abs(target_position - filtered_position) < 5:  # If the error is small enough, stop
                print(f"Position reached: {filtered_position}")
                motor(False)
                return

            # Stop motor and restart every 100 counts, to enable measurement to catch up
            if counter >50:
                motor(False)
            if counter > 200:
                motor(True)
                counter = 0
            
            # print(f"Target position: {target_position}, current: {filtered_position:.0f}, error: {target_position - filtered_position}, counter: {counter}")

# Function to update display with date and distance
def update_display(day, month, year, number, unit, bmp):
    display_group = displayio.Group()  # Create a group to hold display content

    # Add a white rectangle background
    background = Rect(0, 0, display.width, display.height, fill=0xFFFFFF)  # White background
    display_group.append(background)  # Add background to display group first

    print("/bmp/"+bmp)

    # Load the emoji bitmap and position on display
    emoji_bitmap = displayio.OnDiskBitmap("/bmp/"+bmp)  # Place your emoji.bmp file on the device
    emoji_tilegrid = displayio.TileGrid(emoji_bitmap, pixel_shader=emoji_bitmap.pixel_shader)
    emoji_tilegrid.x = 8
    emoji_tilegrid.y = 20
    display_group.append(emoji_tilegrid)

    # Text for the date
    date_text = f"{day}/{month}/{year}"
    print(f"Date string: {date_text}")
    
    # Text for the distance / voltage
    decimal_places = 0 if unit=="km" else 2
    number_text = "{:.{}f}{}".format(number,decimal_places,unit)
    print(f"Number string: {number_text}")

    # Add both date and distance to the display group
    text_area = label.Label(terminalio.FONT, text=date_text, color=0x000000, scale=3)
    text_area.x = 8
    text_area.y = 115
    display_group.append(text_area)

    text_area2 = label.Label(terminalio.FONT, text=number_text, color=0x000000, scale=3)
    text_area2.x = 8
    text_area2.y = 160
    display_group.append(text_area2)

    print("About to sleep for refresh...")

    # Add a delay to avoid refreshing too soon
    time.sleep(display.time_to_refresh)  # Wait for the display to be ready for another refresh

    print("I've slept...")

    display.root_group = display_group  # Set the root group for the display
    display.refresh()  # Refresh the display to show the new content

# Function to calculate the number of seconds until the next target wake-up time (7 PM)
def calculate_seconds_until_wake_up():
    now = time.localtime()  # Get the current local time
    
    target_hour = 19  # 7 PM in 24-hour format
    target_minute = 0
    target_second = 0

    # If it's before 7 PM today
    if now.tm_hour < target_hour:
        hours_until_target = target_hour - now.tm_hour
    # If it's 7 PM or later, calculate how many hours until 7 PM tomorrow
    else:
        hours_until_target = (24 - now.tm_hour) + target_hour

    # Calculate the remaining minutes and seconds
    minutes_until_target = 60 - now.tm_min - 1
    seconds_until_target = 60 - now.tm_sec

    # Total time in seconds until next target time
    total_seconds = (hours_until_target * 3600) + (minutes_until_target * 60) + seconds_until_target
    print(f"Seconds until wake-up time: {total_seconds}")

    return total_seconds

# Function to enter deep sleep setup
def enter_deep_sleep(duration_seconds):
    print(f"Entering deep sleep for {duration_seconds} seconds...")

    # Set a timer alarm to wake up after the specified number of seconds
    print(time.monotonic())
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + duration_seconds)

    # Enter deep sleep until the alarm goes off
    alarm.exit_and_deep_sleep_until_alarms(time_alarm)

def main():
    
    print("Good morning!")

    servo_ena.value = False # Switch off servo
    epd_ena.value = True # Switch on display

    # Check if low battery
    if tinys3.get_battery_voltage() < LOW_BATTERY:
        update_display(current_time.tm_mday, current_time.tm_mon, current_time.tm_year, tinys3.get_battery_voltage(), "V", "charging.bmp")
        epd_ena.value = False
        enter_deep_sleep(3600)
        return

    print(f"5V is: {tinys3.get_vbus_present()}, vbat is: {tinys3.get_battery_voltage()}")
    time.sleep(5)

    coordinates = None # Initiailize coordinates as None to enter the loop
    current_time = None # Initiailize current_time as None to enter the loop
    while not coordinates or not current_time:
        coordinates = get_location_time(config_file_path)
        if coordinates:
            latitude, longitude, accuracy, current_time = coordinates
            print(f"Coordinates: {latitude}, {longitude}, with {accuracy} meters accuracy.")
            print(f"Current time: {current_time}")
        else:
            print("Could not determine location or time, will try again in 10 seconds")
            time.sleep(10)

    if tinys3.get_vbus_present():
        update_display(current_time.tm_mday, current_time.tm_mon, current_time.tm_year, tinys3.get_battery_voltage(), "V", "charging.bmp")
        enter_deep_sleep(600)
        return

    # Find the closest date
    closest_date = find_closest_date(current_time.tm_mon, current_time.tm_mday)

    # Calculate the distance and heading
    target_distance = calculate_distance(latitude, longitude, closest_date["latitude"], closest_date["longitude"])
    target_heading = calculate_heading(latitude, longitude, closest_date["latitude"], closest_date["longitude"])

    print(f"Target Heading: {target_heading:.2f} degrees")
    print(f"Target Distance: {target_distance:.2f} km")

    # Update the e-ink display with the closest date and distance
    update_display(closest_date["day"], closest_date["month"], closest_date["year"], target_distance, "km", closest_date["bmp"])

    # Get the current compass heading from the IMU
    current_heading = get_compass_orientation(declination_angle)
    print(f"Current IMU Heading: {current_heading:.2f} degrees")

    # Calculate the difference between current heading and target heading
    heading_diff = target_heading - current_heading
    if heading_diff < 0: # Adjust for negative heading diff
        heading_diff += 360
    print(f"Heading difference: {heading_diff:.2f} degrees")

    epd_ena.value = False # Switch off display

    servo_ena.value = True # Switch on servo
    set_servo_position(heading_diff + needle_offset) # Set servo to right heading
    # set_servo_position(0) # Uncomment for mounting the needle
    servo_ena.value = False # Switch off servo

    # Enter deep sleep until 7pm tomorrow
    enter_deep_sleep(calculate_seconds_until_wake_up())

while True:
    main()