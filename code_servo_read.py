import time
import digitalio
import board

# Known cycle duration from logic analyzer (in microseconds)

# Servo feedback values
CYCLE_DURATION_NS = 1089000  # 1089 ms or 1.089 seconds
DUTY_CYCLE_MIN = 0.09
DUTY_CYCLE_MAX = 0.84
ALPHA = 0.1
filtered_high_pulse = 0
filtered_low_pulse = 0
filtered_position = 0


counter = 0

def measure_high_pulse(pin):
    # Wait for the pin to go high (start of high pulse)
    while not pin.value:
        pass
    
    # Capture the start time for the high pulse
    start_time = time.monotonic_ns()
    
    # Wait for the pin to go low (end of high pulse)
    while pin.value:
        pass
    
    # Calculate elapsed time for the high pulse in nanoseconds
    elapsed_time_ns = (time.monotonic_ns() - start_time)
    return elapsed_time_ns

def measure_low_pulse(pin):
    # Wait for the pin to go low (start of low pulse)
    while pin.value:
        pass
    
    # Capture the start time for the low pulse
    start_time = time.monotonic_ns()
    
    # Wait for the pin to go high (end of low pulse)
    while not pin.value:
        pass
    
    # Calculate elapsed time for the low pulse in nanoseconds
    elapsed_time_ns = (time.monotonic_ns() - start_time)
    return elapsed_time_ns

# Function to calculate duty cycle based on pulse durations
def calculate_position(high_pulse_duration, low_pulse_duration, cycle_duration_ns):
    if high_pulse_duration > low_pulse_duration:
        duty_cycle = (high_pulse_duration / cycle_duration_ns)
    else:
        duty_cycle = (1 - low_pulse_duration / cycle_duration_ns)
    position = 360 * (duty_cycle - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)
    if position < 0:
        position = 0
    elif position > 360:
        position = 360
    return position

# Function to apply low-pass filtering (exponential moving average)
def low_pass_filter(previous_value, new_value, alpha):
    return previous_value * (1 - alpha) + new_value * alpha

# Example usage with a pin
feedback_pin = digitalio.DigitalInOut(board.D0)
feedback_pin.direction = digitalio.Direction.INPUT

while True:
    # Measure the high and low pulse durations
    high_pulse = measure_high_pulse(feedback_pin)
    low_pulse = measure_low_pulse(feedback_pin)
    
    # Calculate the duty cycle based on the high pulse and known cycle duration
    position = calculate_position(high_pulse, low_pulse, CYCLE_DURATION_NS)
    
    # Apply low-pass filtering to the high pulse, low pulse, and duty cycle
    filtered_high_pulse = low_pass_filter(filtered_high_pulse, high_pulse, ALPHA)
    filtered_low_pulse = low_pass_filter(filtered_low_pulse, low_pulse, ALPHA)
    filtered_position = low_pass_filter(filtered_position, position, ALPHA)   
    
    counter += 1

    if counter > 100:
        # Print the filtered values
        print(f"Position: {filtered_position:.2f}, High pulse: {filtered_high_pulse/1000:.0f} µs, Low pulse: {filtered_low_pulse/1000:.0f} µs ")
        counter = 0
