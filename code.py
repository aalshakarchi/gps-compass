import time
import math
import board
import busio
import displayio
from adafruit_ssd1681 import SSD1681
import adafruit_gps
import adafruit_icm20x

# Import the date data from dates.py
from dates import date_coordinates

# Setup for E-Ink Display
displayio.release_displays()

spi = busio.SPI(board.SCK, board.MOSI)
epd_cs = board.D10  # Adjust based on your actual pin config
epd_dc = board.D9
display_bus = displayio.FourWire(spi, command=epd_dc, chip_select=epd_cs)

# Initialize the e-ink display with 200x200 pixels resolution
display = SSD1681(display_bus, width=200, height=200)

# Setup for GPS (UART)
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)

# Turn on the basic GGA and RMC info
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')  # 1 Hz update rate

# Setup for IMU (I2C)
i2c = busio.I2C(board.SCL, board.SDA)
icm = adafruit_icm20x.ICM20948(i2c)

# Magnetic declination for your location (in degrees)
declination_angle = 2.12  # You can adjust this value based on your location

# Earth radius in kilometers
EARTH_RADIUS_KM = 6371.0

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

    display.show(display_group)  # Show the group on the display
    display.refresh()  # Refresh the display to show the new content

def main():
    # Wait for GPS to get a fix
    gps.update()

    if not gps.has_fix:
        print("Waiting for GPS fix...")
        time.sleep(1)
        return
    
    # Retrieve current GPS coordinates and date
    gps_lat = gps.latitude
    gps_lon = gps.longitude
    gps_month = gps.datetime_month
    gps_day = gps.datetime_day

    # Find the closest date
    closest_date = find_closest_date(gps_month, gps_day)

    # Calculate the distance and heading
    distance = calculate_distance(gps_lat, gps_lon, closest_date["latitude"], closest_date["longitude"])
    heading = calculate_heading(gps_lat, gps_lon, closest_date["latitude"], closest_date["longitude"])

    # Update the e-ink display with the closest date and distance
    update_display(closest_date["day"], closest_date["month"], closest_date["year"], distance)

    print(f"Target Heading: {heading:.2f} degrees")
    print(f"Target Distance: {distance:.2f} km")
    
    # Sleep to save power (adjust to your needs)
    time.sleep(5)

while True:
    main()