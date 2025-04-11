from machine import UART, Pin, I2C
import time
import math
from mlx90614 import MLX90614

# Initialize UART for Nextion display
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Initialize temperature sensor
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)
sensor = MLX90614(i2c)

# Pulse control setup
INPUT_PIN = 2  # Using GP2 (Pin 4) to avoid I2C conflict
OUTPUT_PIN = 9  # GP9 (Pin 12)
COOLING_PIN = 15  # GP15 (Pin 20) for cooling control
input_pin = Pin(INPUT_PIN, Pin.IN, Pin.PULL_UP)
output_pin = Pin(OUTPUT_PIN, Pin.OUT)
cooling_pin = Pin(COOLING_PIN, Pin.OUT)
output_pin.value(0)
cooling_pin.value(0)  # Start with cooling off

# Waveform settings
WAVEFORM_ID = 5        # Waveform component ID
PROFILE_CHANNEL = 0    # Channel for target profile (red)
MEASURED_CHANNEL = 1   # Channel for measured temp (green)
MAX_TEMP = 350         # Maximum temperature for scaling
CMD_END = b'\xff\xff\xff'

# Global variables
current_edit_context = ""  # Stores which value is being edited
edit_values = {}  # Dictionary to store the values
pulse_detected = False
start = False  # Start signal initially off
heating_active = False  # Track if heating is active

# System parameters
MAX_DELAY = 9.0  # Maximum delay in ms (half cycle at 60Hz)
MIN_DELAY = 0.1  # Minimum delay in ms
SAMPLE_INTERVAL = 1.0  # Time between temperature measurements (seconds)
COOLING_THRESHOLD = 35  # Temperature threshold to turn off cooling

# Temperature profile parameters (default values)
m1 = 2.0  # Slope 1 (°C/s)
m2 = 0.0  # Slope 2 (°C/s)
m3 = 1.0  # Slope 3 (°C/s)
m4 = 0.0  # Slope 4 (°C/s) - negative for cooling
t1 = 80  # Time for slope 1 (seconds)
t2 = 95  # Time for slope 2 (seconds)
t3 = 65  # Time for slope 3 (seconds)
t4 = 10  # Time for slope 4 (seconds)

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, current_value):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.01  # prevent division by zero
        
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt
        D = self.Kd * derivative
        
        # Remember last error and time
        self.last_error = error
        self.last_time = now
        
        # Calculate output
        output = P + I + D
        
        return output
        
    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        # Reset integral term when setpoint changes to prevent windup
        self.integral = 0

# Initialize PID controller for slope control
pid = PIDController(Kp=100.0, Ki=1.5, Kd=0.0, setpoint=m1)

def input_callback(pin):
    global pulse_detected
    if start:  # Only detect pulses when system is started
        pulse_detected = True

input_pin.irq(trigger=Pin.IRQ_RISING, handler=input_callback)

def send_nextion_command(command):
    """Send a command to the Nextion display"""
    uart.write(command.encode())
    uart.write(CMD_END)
    time.sleep(0.1)  # Give display time to process

def set_nextion_text(component_id, text):
    """Set text of a component on Nextion display"""
    command = f'{component_id}.txt="{text}"'
    send_nextion_command(command)
        
def navigate_to_page(page_num):
    """Navigate to a specific page on the Nextion display"""
    send_nextion_command(f'page {page_num}')

def scale_temp_to_value(temp):
    """Scale temperature to 0-255 range for waveform"""
    value = int((temp / MAX_TEMP) * 255)
    return min(255, max(0, value))

def get_profile_temp(elapsed):
    """Calculate temperature based on the profile and elapsed time"""
    if elapsed < t1:  # Phase 1: 0-t1s, rising slope m1
        return 25 + m1 * elapsed
    elif elapsed < t1 + t2:  # Phase 2: t1-(t1+t2)s, slope m2
        return 25 + m1*t1 + m2*(elapsed - t1)
    elif elapsed < t1 + t2 + t3:  # Phase 3: (t1+t2)-(t1+t2+t3)s, slope m3
        return 25 + m1*t1 + m2*t2 + m3*(elapsed - t1 - t2)
    elif elapsed < t1 + t2 + t3 + t4:  # Phase 4: (t1+t2+t3)-(t1+t2+t3+t4)s, slope m4
        return 25 + m1*t1 + m2*t2 + m3*t3 + m4*(elapsed - t1 - t2 - t3)
    else:  # After profile is complete
        return 25 + m1*t1 + m2*t2 + m3*t3 + m4*t4

def setup_waveform():
    """Configure the waveform display with two channels"""
    navigate_to_page(1)
    send_nextion_command('cle 5,0')        # Clear Channel 0 (profile)
    send_nextion_command('cle 5,1')        # Clear Channel 1 (measured)
    send_nextion_command('s0.pco0=63488')  # Red for profile
    send_nextion_command('s0.pco1=2016')   # Green for measured
    send_nextion_command('s0.aph=0')       # Make waveform visible
    send_nextion_command('ref s0')

def update_display(profile_temp, measured_temp, elapsed_time):
    """Update both the profile and measured temperatures on display"""
    
    total_profile_time = t1 + t2 + t3 + t4
    remaining_time = max(0, total_profile_time - elapsed_time)
    # Update numeric displays
    set_nextion_text("n0", f"{measured_temp:.1f}")  # Measured temp with 1 decimal
    set_nextion_text("n1", f"{remaining_time:.0f}")  # Remaining time in seconds
    
    # Scale temperatures for waveform
    profile_value = scale_temp_to_value(profile_temp)
    measured_value = scale_temp_to_value(measured_temp)
    
    # Add points to waveforms
    send_nextion_command(f'add {WAVEFORM_ID},{PROFILE_CHANNEL},{profile_value}')
    send_nextion_command(f'add {WAVEFORM_ID},{MEASURED_CHANNEL},{measured_value}')
    send_nextion_command('ref s0')
    
    # Print to console
    print(f"Time: {time.time()-profile_start_time:.1f}s | Profile: {profile_temp:.1f}°C | Measured: {measured_temp:.1f}°C")

def get_calibrated_temp():
    """Get and calibrate temperature reading"""
    raw_temp = sensor.object_temp
    if raw_temp is None or math.isnan(raw_temp):
        return None
    
    # Apply base calibration formula
    calibrated_temp = 1.219 * raw_temp - 7.187
    
    # Progressive additional calibration at higher temperatures
    thresholds = range(160, 261, 5)  # 160, 165, 170,..., 260
    increments = sum(1 for threshold in thresholds if calibrated_temp > threshold)
    calibrated_temp += increments * 2.5
    
    return calibrated_temp

def generate_pulse():
    if start:  # Only generate pulses when system is started
        output_pin.value(1)
        time.sleep_ms(1)
        output_pin.value(0)
    else:
        output_pin.value(0)

def generate_delayed_pulse(delay_ms):
    """Generate output pulse after specified delay from zero-crossing"""
    if start:  # Only generate pulses when system is started
        if delay_ms < MAX_DELAY:
            time.sleep_ms(int(delay_ms))  # Ensure delay_ms is integer
            generate_pulse()
        else:
            # Skip this half-cycle entirely
            pass
    else:
        output_pin.value(0)

def calculate_slope(temps):
    """Calculate current temperature slope from recent measurements"""
    if len(temps) < 2:
        return 0.0
    
    # Use last few measurements for slope calculation
    num_points = min(5, len(temps))
    recent_temps = temps[-num_points:]
    
    time_diff = recent_temps[-1][0] - recent_temps[0][0]
    temp_diff = recent_temps[-1][1] - recent_temps[0][1]
    
    if time_diff > 0:
        return temp_diff / time_diff
    return 0.0

def start_heating_process():
    """Start the heating process with current profile settings"""
    global start, heating_active, pulse_detected, profile_start_time
    
    print("Starting heating process")
    heating_active = True
    start = True
    pulse_detected = False
    profile_start_time = time.time()
    
    # Show countdown page
    navigate_to_page(8)
    time.sleep(3)
    
    # Setup waveform with two channels
    setup_waveform()
    navigate_to_page(1)
    
    temps = []
    last_update = time.time()
    current_stage = 1
    delay_ms = MAX_DELAY / 2  # Initial delay
    cooling_active = False
    
    print("System started!")
    print("Time(s)\tProfile(°C)\tActual(°C)\tSlope(°C/s)\tDelay(ms)")
    
    while heating_active:
        current_time = time.time()
        elapsed_time = current_time - profile_start_time-4
        
        # Determine current stage based on elapsed time
        if elapsed_time < t1:
            if current_stage != 1:
                current_stage = 1
                pid.set_setpoint(m1)
        elif elapsed_time < t1 + t2:
            if current_stage != 2:
                current_stage = 2
                pid.set_setpoint(m2)
        elif elapsed_time < t1 + t2 + t3:
            if current_stage != 3:
                current_stage = 3
                pid.set_setpoint(m3)
        elif elapsed_time < t1 + t2 + t3 + t4:
            if current_stage != 4:
                current_stage = 4
                pid.set_setpoint(m4)
        else:
            if current_stage != 5:
                current_stage = 5
                start = False
                pid.set_setpoint(0)
                cooling_pin.value(1)
                cooling_active = True
        
        # Handle incoming pulses (only when start is True)
        if pulse_detected and start:
            pulse_detected = False
            current_slope = calculate_slope(temps)
            pid_output = pid.update(current_slope)
            delay_ms = max(MIN_DELAY, min(MAX_DELAY, MAX_DELAY * (1 - pid_output/100)))
            generate_delayed_pulse(delay_ms)
        
        # Take temperature measurements at regular intervals
        if current_time - last_update >= SAMPLE_INTERVAL:
            try:
                temp = get_calibrated_temp()
                if temp is not None and not math.isnan(temp):
                    temps.append((current_time, temp))
                    
                    # Calculate appropriate profile temperature
                    if current_stage == 5:
                        profile_temp = temp  # During cooling, show actual temp as profile
                    else:
                        profile_temp = get_profile_temp(elapsed_time)
                    
                    # Single display update
                    update_display(profile_temp, temp, elapsed_time)
                    
                    # Check cooling threshold
                    if current_stage == 5 and cooling_active and temp < COOLING_THRESHOLD:
                        cooling_pin.value(0)
                        cooling_active = False
                        heating_active = False
                        navigate_to_page(9)
                        print("Cooling complete - system stopped")
                
                last_update = current_time
            except Exception as e:
                print(f"Error reading temperature: {e}")
        
        time.sleep_ms(10)

# Rest of your existing code for touch handling and main loop...
def read_nextion_touch():
    """Read and process touch event data from Nextion display"""
    if uart.any():
        time.sleep(0.05)  # Give time for all data to arrive
        data = uart.read()
        if data:
            print(f"Raw data: {data.hex()}")
            
            # Look for touch event pattern: 0x65 page_id component_id action_type
            i = 0
            while i < len(data) - 4:  # Need at least 4 bytes for a touch event
                if data[i] == 0x65:  # Touch event code
                    page_id = data[i+1]
                    component_id = data[i+2]
                    action_type = data[i+3]
                    print(f"Touch event: page={page_id}, component={component_id}, action={action_type}")
                    return (page_id, component_id, action_type)
                i += 1
                
    return None


def get_button_name(component_id):
    """Map component IDs to button names"""
    # Adjust these mappings based on your actual Nextion design
    buttons = {
        3: "b0",  # Button with ID 3 on page 10
        # Add other button mappings as needed
    }
    return buttons.get(component_id, None)

def main_loop():
    """Main program loop"""
    global current_edit_context, start, heating_active
    
    # Initialize with default values
    edit_values.update({
        "m1": m1, "t1": t1,
        "m2": m2, "t2": t2,
        "m3": m3, "t3": t3,
        "m4": m4, "t4": t4
    })
    
    while True:
        touch_event = read_nextion_touch()
        if touch_event:
            page_id, component_id, action_type = touch_event
            
            # Check if start button (b0 with id 3 on page10) was pressed
            if page_id == 10 and component_id == 3 and action_type == 1:
                print("Start button pressed - beginning heating process")
                start_heating_process()
            
            # Add other touch event handling as needed
            
        time.sleep(0.1)  # Small delay to prevent CPU hogging

# Start the program
if __name__ == "__main__":
    print("Starting integrated control program")
    # Initialize display with default values
    for key, value in edit_values.items():
        set_nextion_text(key, str(value))
    
    # Start main loop
    main_loop()

