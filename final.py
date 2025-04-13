import time
import machine
import math
from mlx90614 import MLX90614
from machine import Pin, I2C, UART

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
# Add this with other pin initializations
LID_SENSOR_PIN = 16  # GP16 (Pin 21) for lid sensor
lid_sensor = Pin(LID_SENSOR_PIN, Pin.IN, Pin.PULL_UP)

# Waveform settings
WAVEFORM_ID = 7        # Waveform component ID
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
profile_start_time = 0  # Will be set when heating starts

# System parameters
MAX_DELAY = 9.9  # Maximum delay in ms (half cycle at 60Hz)
MIN_DELAY = 0.1  # Minimum delay in ms
SAMPLE_INTERVAL = 1.0  # Time between temperature measurements (seconds)
COOLING_THRESHOLD = 80  # Temperature threshold to turn off cooling
cooling_start_time = None
cooling_start_temp = None

# Temperature profile parameters (default values)
m1 = 2.0  # Slope 1 (°C/s)
m2 = 0.0  # Slope 2 (°C/s)
m3 = 1.0  # Slope 3 (°C/s)
m4 = 0.0  # Slope 4 (°C/s) - negative for cooling
t1 = 80  # Time for slope 1 (seconds)
t2 = 95  # Time for slope 2 (seconds)
t3 = 65  # Time for slope 3 (seconds)
t4 = 10  # Time for slope 4 (seconds)

# Nextion Display Functions
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
    # If we're going to the main page, update all values quickly
    if page_num == 9:
        # No delay here - send commands immediately
        update_all_display_values(fast=True)

def scale_temp_to_value(temp):
    """Scale temperature to 0-255 range for waveform"""
    value = int((temp / MAX_TEMP) * 255)
    return min(255, max(0, value))

def setup_waveform():
    """Configure the waveform display with two channels"""
    navigate_to_page(1)
    send_nextion_command('cle 7,0')        # Clear Channel 0 (profile)
    send_nextion_command('cle 7,1')        # Clear Channel 1 (measured)
    send_nextion_command('s0.pco0=63488')  # Red for profile
    send_nextion_command('s0.pco1=2016')   # Green for measured
    send_nextion_command('s0.aph=0')       # Make waveform visible
    send_nextion_command('ref s0')

def update_display(profile_temp, measured_temp, elapsed_time):
    """Update both the profile and measured temperatures on display"""
    total_profile_time = t1 + t2 + t3 + t4
    remaining_time = max(0, total_profile_time+180 - elapsed_time)
    
    # Update numeric displays
    
    send_nextion_command(f'n0.val={int(measured_temp)}')  # Remaining time in seconds
    send_nextion_command(f'n1.val={int(remaining_time)}')  # Remaining time in seconds
    
    # Scale temperatures for waveform
    profile_value = scale_temp_to_value(profile_temp)
    measured_value = scale_temp_to_value(measured_temp)
    
    # Add points to waveforms
    send_nextion_command(f'add {WAVEFORM_ID},{PROFILE_CHANNEL},{profile_value}')
    send_nextion_command(f'add {WAVEFORM_ID},{MEASURED_CHANNEL},{measured_value}')
    send_nextion_command('ref s0')
    
    # Print to console
    print(f"Time: {elapsed_time:.1f}s | Profile: {profile_temp:.1f}°C | Measured: {measured_temp:.1f}°C")

def update_all_display_values(fast=False):
    """Update all text fields with current values"""
    # First prepare and send all commands in a batch
    for key, value in edit_values.items():
        print(f"Setting {key} to {value}")
        # Format the value - use integers when possible
        display_value = str(int(value)) if value == int(value) else str(value)
        send_nextion_command(f'{key}.txt="{display_value}"')
        
        if not fast:
            # Only add delay in slow mode
            time.sleep(0.05)
    
    # Force a refresh of all components - but only if not in fast mode
    if not fast:
        for key in edit_values:
            send_nextion_command(f'ref {key}')
            time.sleep(0.05)


def get_calibrated_temp():
    """Get and calibrate temperature reading"""
    raw_temp = sensor.object_temp
    if raw_temp is None or math.isnan(raw_temp):
        return None
    
    # Apply base calibration formula
    calibrated_temp = 1.519 * raw_temp - 4.187
    
    # Progressive additional calibration at higher temperatures (in 10°C increments)
    if calibrated_temp > 300:
        calibrated_temp += 36
    elif calibrated_temp > 290:
        calibrated_temp += 36
    elif calibrated_temp > 280:
        calibrated_temp += 34
    elif calibrated_temp > 270:
        calibrated_temp += 32
    elif calibrated_temp > 260:
        calibrated_temp += 30
    elif calibrated_temp > 250:
        calibrated_temp += 28
    elif calibrated_temp > 240:
        calibrated_temp += 26
    elif calibrated_temp > 230:
        calibrated_temp += 24
    elif calibrated_temp > 220:
        calibrated_temp += 22
    elif calibrated_temp > 210:
        calibrated_temp += 20
    elif calibrated_temp > 200:
        calibrated_temp += 10
    elif calibrated_temp > 190:
        calibrated_temp += 8
    elif calibrated_temp > 180:
        calibrated_temp += 6
    elif calibrated_temp > 170:
        calibrated_temp += 4
    elif calibrated_temp > 160:
        calibrated_temp += 2
    
    return calibrated_temp

# Modify the get_profile_temp function to handle cooling
def get_profile_temp(elapsed, cooling_start_time=None, cooling_start_temp=None):
    """Calculate temperature based on the profile and elapsed time"""
    # Handle cooling phase if cooling parameters are provided
    if cooling_start_time is not None and cooling_start_temp is not None:
        cooling_elapsed = time.time() - cooling_start_time
        return cooling_start_temp - 0.5 * cooling_elapsed  # -0.5°C per second

    # Regular profile calculations (unchanged)
    if elapsed < t1:  # Phase 1: 0-t1s, rising slope m1
        return m1 * elapsed
    elif elapsed < t1 + t2:  # Phase 2: t1-(t1+t2)s, slope m2
        return m1*t1 + m2*(elapsed - t1)
    elif elapsed < t1 + t2 + t3:  # Phase 3: (t1+t2)-(t1+t2+t3)s, slope m3
        return m1*t1 + m2*t2 + m3*(elapsed - t1 - t2)
    elif elapsed < t1 + t2 + t3 + t4:  # Phase 4: (t1+t2+t3)-(t1+t2+t3+t4)s, slope m4
        return m1*t1 + m2*t2 + m3*t3 + m4*(elapsed - t1 - t2 - t3)
    else:  # After profile is complete
        return m1*t1 + m2*t2 + m3*t3 + m4*t4

# PID Controller
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

# Pulse Control Functions
def input_callback(pin):
    global pulse_detected
    if start:  # Only detect pulses when system is started
        pulse_detected = True

input_pin.irq(trigger=Pin.IRQ_RISING, handler=input_callback)

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
    
    # Initialize cooling variables
    cooling_start_time = None
    cooling_start_temp = None
    
    print("System started!")
    print("Stage\tTime(s)\tTemp(°C)\tSlope(°C/s)\tTarget Slope\tDelay(ms)")
    
    while heating_active:
        current_time = time.time()
        elapsed_time = current_time - profile_start_time
        
        # Determine current stage based on elapsed time
        if elapsed_time < t1:
            if current_stage != 1:
                current_stage = 1
                pid.set_setpoint(m1)
                print(f"Entering Stage 1: Target slope = {m1}°C/s")
        elif elapsed_time < t1 + t2:
            if current_stage != 2:
                current_stage = 2
                pid.set_setpoint(m2)
                print(f"Entering Stage 2: Target slope = {m2}°C/s")
        elif elapsed_time < t1 + t2 + t3:
            if current_stage != 3:
                current_stage = 3
                pid.set_setpoint(m3)
                print(f"Entering Stage 3: Target slope = {m3}°C/s")
        elif elapsed_time < t1 + t2 + t3 + t4:
            if current_stage != 4:
                current_stage = 4
                pid.set_setpoint(m4)
                print(f"Entering Stage 4: Target slope = {m4}°C/s")
        else:
            if current_stage != 5:
                current_stage = 5
                start = False  # Turn off start signal after t4
                pid.set_setpoint(0)  # Maintain temperature
                print("Profile complete - system stopped")
                
                # Activate cooling only after all slopes are done
                cooling_pin.value(1)
                cooling_active = True
                print("Cooling activated (GPIO 15 ON)")
        
        # Handle incoming pulses (only when start is True)
        if pulse_detected and start:
            pulse_detected = False
            
            # Calculate current slope
            current_slope = calculate_slope(temps)
            
            # Update PID controller with current slope
            pid_output = pid.update(current_slope)
            
            # Calculate delay based on PID output (scaled to 0-MAX_DELAY range)
            delay_ms = max(MIN_DELAY, min(MAX_DELAY, MAX_DELAY * (1 - pid_output/100)))
            
            # Generate pulse with calculated delay
            generate_delayed_pulse(delay_ms)
        
        # Take temperature measurements at regular intervals
        if current_time - last_update >= SAMPLE_INTERVAL:
            try:
                temp = get_calibrated_temp()
                if temp is not None and not math.isnan(temp):
                    temps.append((current_time, temp))
                    
                    # Calculate appropriate profile temperature
                    if current_stage == 5:
                        # Initialize cooling phase if just entered
                        if cooling_start_time is None:
                            cooling_start_time = current_time
                            cooling_start_temp = temp
                            print(f"Starting cooling from {temp}°C")
                        
                        # Calculate elapsed time since cooling started (in seconds)
                        cooling_elapsed = current_time - cooling_start_time
                        
                        # Calculate target temperature with slope of -0.5°C per second
                        profile_temp = cooling_start_temp - 0.5 * cooling_elapsed
                        print(f"Cooling: {temp}°C, Profile: {profile_temp}°C, Slope: -0.5°C/s")
                    else:
                        profile_temp = get_profile_temp(elapsed_time)
                    
                    # Update display with both curves
                    update_display(profile_temp, temp, elapsed_time)
                    
                    # Check cooling threshold
                    if current_stage == 5 and cooling_active and temp < COOLING_THRESHOLD:
                        cooling_pin.value(0)
                        cooling_active = False
                        heating_active = False
                        navigate_to_page(7)
                        print("Cooling complete - system stopped")
                        # Continue plotting for a while after stopping
                        for _ in range(10):  # Continue for 10 more samples
                            temp = get_calibrated_temp()
                            if temp is not None:
                                # Calculate continued cooling profile
                                cooling_elapsed = time.time() - cooling_start_time
                                profile_temp = cooling_start_temp - 0.5 * cooling_elapsed
                                update_display(profile_temp, temp, elapsed_time)
                                time.sleep(SAMPLE_INTERVAL)
                
                last_update = current_time
            except Exception as e:
                print(f"Error reading temperature: {e}")
        
        time.sleep_ms(10)

# Touch Event Handling
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
    # Original button mappings
    buttons = {
        3: "b0",   # Button with ID 3 on page 10
        10: "b8",  # Button with ID 10 on page 9
        # Add other button mappings as needed
    }
    
    # New T/t button mappings
    t_buttons = {
        2: "T0", 3: "T1", 4: "T2", 5: "T3", 
        6: "T4", 7: "T5", 8: "T6"
    }
    T_buttons = {
        10: "t0", 11: "t1", 12: "t2", 13: "t3",
        14: "t4", 15: "t5", 16: "t6"
    }
    
    # Combine all dictionaries
    all_buttons = {}
    all_buttons.update(buttons)    # Original buttons
    all_buttons.update(t_buttons)  # T buttons
    all_buttons.update(T_buttons)  # t buttons
    
    return all_buttons.get(component_id, None)

def update_plot_point():
    """Update the text and position of the edited point on the plot"""
    global current_edit_context
    
    if current_edit_context and current_edit_context in edit_values:
        value = edit_values[current_edit_context]
        print(f"Updating {current_edit_context} display with value {value}")
        
        # We'll update just this one component first
        # Approach 1: Standard text update
        set_nextion_text(current_edit_context, str(value))
        
        # Approach 2: Use a different format - try without decimals if it's a whole number
        if value == int(value):
            set_nextion_text(current_edit_context, str(int(value)))
        
        # Force refresh the component
        send_nextion_command(f'ref {current_edit_context}')
        
        # Update plot if needed
        update_plot_display()

def update_plot_display():
    """Update the plot display based on current values"""
    # Here you would calculate points and update visual elements
    # Implementation depends on your specific Nextion design
    pass

def handle_keypad_entry():
    """Handle keypad entry on Page 4"""
    global current_edit_context
    
    input_buffer = ""
    set_nextion_text("tValue", "")  # Clear the display field
    
    print(f"Handling keypad entry for {current_edit_context}")
    
    while True:
        touch_event = read_nextion_touch()
        if touch_event:
            page_id, component_id, action_type = touch_event
            print(f"Keypad touch: page={page_id}, component={component_id}")
            
            # Check if we're on the keypad page (page 4)
            if page_id == 4:
                # Process keypad touches based on your actual component IDs
                # Numeric buttons (adjust these component IDs to match your Nextion design)
                if 2 <= component_id <= 11:  # Assuming these are digit buttons 0-9
                    digit = component_id - 2  # Map component ID to digit
                    if digit == 10:  # Handle special case for zero
                        digit = 0
                    input_buffer += str(digit)
                    set_nextion_text("tValue", input_buffer)
                    print(f"Added digit {digit}, buffer: {input_buffer}")
                    
                # Delete/Backspace button
                elif component_id == 13:  # This is your bDel button
                    print("Delete button pressed")
                    if input_buffer:
                        # Remove the last character
                        input_buffer = input_buffer[:-1]
                        set_nextion_text("tValue", input_buffer)
                        print(f"Deleted last digit, buffer: {input_buffer}")
                        
                # Done button
                elif component_id == 14:  # This matches your observed component ID
                    print("Done button pressed")
                    if input_buffer:
                        try:
                            value = float(input_buffer)
                            
                            # Validate value based on T or t context
                            is_valid = True
                            if current_edit_context.startswith('T') and value > 250:
                                is_valid = False
                                set_nextion_text("tValue", "Invalid value")
                            elif current_edit_context.startswith('t') and value > 420:
                                is_valid = False
                                set_nextion_text("tValue", "Invalid value") 
                            
                            if is_valid:
                                edit_values[current_edit_context] = value
                                print(f"Saving value {value} for {current_edit_context}")
                                print(f"All current values: {edit_values}")
                                
                                # First update this specific component's text for speed
                                set_nextion_text(current_edit_context, str(value))
                                
                                # Then navigate to page 9
                                navigate_to_page(9)
                                return value
                        except ValueError:
                            set_nextion_text("tValue", "Error")
                            time.sleep(1)
                            set_nextion_text("tValue", input_buffer)
                
                # Cancel button (adjust ID)
                elif component_id == 16:  # Adjust this to your Cancel button ID
                    print("Cancel button pressed")
                    # Clear the input field instead of navigating back
                    input_buffer = ""
                    set_nextion_text("tValue", "")
        
        time.sleep(0.05)  # Small delay to prevent CPU hogging

def calculate_slopes_from_points():
    """Calculate slopes between each pair of points based on T/t values"""
    slopes = []
    times = []
    
    # Get all the values from edit_values
    T_values = [edit_values[f"T{i}"] for i in range(7)]
    t_values = [edit_values[f"t{i}"] for i in range(7)]
    
    # Calculate slopes between each consecutive pair of points
    for i in range(4):
        delta_T = T_values[i+1] - T_values[i]
        delta_t = t_values[i+1] - t_values[i]
        
        # Avoid division by zero (if times are equal, slope is 0)
        if delta_t > 0:
            slope = delta_T / delta_t
        else:
            slope = 0
        
        slopes.append(slope)
        times.append(delta_t)
    
    return slopes, times

def start_heating_process2():
    """Start heating process using custom profile from T/t points"""
    # Avoid variable naming conflicts by using a different module name
    import time as time_module  
    global start, heating_active, pulse_detected, profile_start_time
    print("ENTERING start_heating_process2()")
    print(f"edit_values: {edit_values}")  # Debug

    # Calculate slopes and times
    slopes, segment_times = calculate_slopes_from_points()
    print(f"Slopes: {slopes}, Segment Times: {segment_times}")

    print("Starting custom heating process with calculated slopes:")
    print("Segment | Time (s) | Slope (°C/s)")
    for i, (slope, time_val) in enumerate(zip(slopes, segment_times)):
        print(f"{i+1:6} | {time_val:8.1f} | {slope:10.2f}")
    
    heating_active = True
    start = True
    pulse_detected = False
    profile_start_time = time_module.time()  # Use time_module instead of time
    
    # Show countdown page
    navigate_to_page(8)
    time_module.sleep(3)  # Use time_module instead of time
    
    # Setup waveform with two channels
    setup_waveform()
    navigate_to_page(1)
    
    temps = []
    last_update = time_module.time()  # Use time_module instead of time
    current_stage = 0
    delay_ms = MAX_DELAY / 2  # Initial delay
    cooling_active = False
    accumulated_time = 0
    
    # Initialize cooling variables outside the loop
    cooling_start_time = None
    cooling_start_temp = None
    
    print("System started with custom profile!")
    print("Stage\tTime(s)\tTemp(°C)\tSlope(°C/s)\tTarget Slope\tDelay(ms)")
    
    while heating_active:
        current_time = time_module.time()  # Use time_module instead of time
        elapsed_time = current_time - profile_start_time
        
        # Determine current stage based on elapsed time
        stage_found = False
        accumulated_time = 0
        
        for i in range(len(segment_times)):
            stage_end = accumulated_time + segment_times[i]
            if elapsed_time < stage_end:
                if current_stage != i:
                    current_stage = i
                    pid.set_setpoint(slopes[i])
                    print(f"Entering Stage {i+1}: Target slope = {slopes[i]:.2f}°C/s")
                stage_found = True
                break
            accumulated_time = stage_end
        
        # If we've passed all stages, enter cooling phase
        if not stage_found and current_stage != len(slopes):
            current_stage = len(slopes)
            start = False
            pid.set_setpoint(0)
            cooling_pin.value(1)
            cooling_active = True
            print("Custom profile complete - entering cooling phase")
        
        # Handle incoming pulses (only when start is True)
        if pulse_detected and start:
            pulse_detected = False
            
            # Calculate current slope
            current_slope = calculate_slope(temps)
            
            # Update PID controller with current slope
            pid_output = pid.update(current_slope)
            
            # Calculate delay based on PID output (scaled to 0-MAX_DELAY range)
            delay_ms = max(MIN_DELAY, min(MAX_DELAY, MAX_DELAY * (1 - pid_output/100)))
            
            # Generate pulse with calculated delay
            generate_delayed_pulse(delay_ms)
        
        # Take temperature measurements at regular intervals
        if current_time - last_update >= SAMPLE_INTERVAL:
            try:
                temp = get_calibrated_temp()
                if temp is not None and not math.isnan(temp):
                    temps.append((current_time, temp))
                    
                    # Calculate appropriate profile temperature
                    if current_stage == len(slopes):  # In cooling phase
                        # Initialize cooling phase if just entered
                        if cooling_start_time is None:
                            cooling_start_time = current_time
                            cooling_start_temp = temp
                            print(f"Starting cooling from {temp}°C")
                        
                        # Calculate elapsed time since cooling started (in seconds)
                        cooling_elapsed = current_time - cooling_start_time
                        
                        # Calculate target temperature with slope of -0.5°C per second
                        profile_temp = cooling_start_temp - 0.5 * cooling_elapsed
                        print(f"Cooling: {temp}°C, Profile: {profile_temp}°C, Slope: -0.5°C/s")
                    else:
                        # Calculate temperature target based on position in the profile
                        # Start with the first point temperature
                        T_values = [edit_values[f"T{i}"] for i in range(7)]
                        t_values = [edit_values[f"t{i}"] for i in range(7)]
                        
                        # Find where we are in the profile
                        profile_time = elapsed_time
                        
                        # Calculate current target temperature based on elapsed time
                        if profile_time <= t_values[0]:
                            # Before first point, linear from 0 to first point
                            profile_temp = (T_values[0] / t_values[0]) * profile_time
                        else:
                            # Find which segment we're in
                            segment_idx = 0
                            segment_time = 0
                            
                            for i in range(len(segment_times)):
                                if profile_time <= segment_time + segment_times[i]:
                                    segment_idx = i
                                    segment_progress = profile_time - segment_time
                                    # Linear interpolation between points
                                    segment_ratio = segment_progress / segment_times[i]
                                    profile_temp = T_values[i] + segment_ratio * (T_values[i+1] - T_values[i])
                                    break
                                segment_time += segment_times[i]
                            else:
                                # After last point, maintain final temperature
                                profile_temp = T_values[-1]
                    
                    # Update display with both curves
                    update_display(profile_temp, temp, elapsed_time)
                    
                    # Check cooling threshold
                    if current_stage == len(slopes) and cooling_active and temp < COOLING_THRESHOLD:
                        cooling_pin.value(0)
                        cooling_active = False
                        heating_active = False
                        navigate_to_page(7)
                        print("Cooling complete - system stopped")
                        
                        # Continue plotting for a while after stopping
                        for _ in range(10):  # Continue for 10 more samples
                            temp = get_calibrated_temp()
                            if temp is not None:
                                # After cooling stops, continue showing the cooling profile
                                cooling_elapsed = time_module.time() - cooling_start_time
                                profile_temp = cooling_start_temp - 0.5 * cooling_elapsed
                                update_display(profile_temp, temp, elapsed_time)
                                time_module.sleep(SAMPLE_INTERVAL)
                    
                    last_update = current_time
            except Exception as e:
                print(f"Error reading temperature: {e}")
        
        time_module.sleep_ms(10)

# Main Program Loop
def main_loop():
    """Main program loop"""
    global current_edit_context, start, heating_active, m1, m2, m3, m4, t1, t2, t3, t4
    
    # Initialize with default values
    edit_values.update({
        "m1": m1, "t1": t1,
        "m2": m2, "t2": t2,
        "m3": m3, "t3": t3,
        "m4": m4, "t4": t4
    })
    
    # Initialize display with default values
    for key, value in edit_values.items():
        set_nextion_text(key, str(value))
    
    while True:
        touch_event = read_nextion_touch()
        if touch_event:
            page_id, component_id, action_type = touch_event
            
            # Handle start buttons (from either page 10 or page 9)
            if (page_id == 10 and component_id == 3 and action_type == 1) or \
               (page_id == 9 and component_id == 9 and action_type == 1):
                print("Start button pressed - checking lid status")
                
                # Check lid status (pin 16: high when open, low when closed)
                if lid_sensor.value() == 1:  # Lid is open
                    print("Lid is open - showing warning")
                    navigate_to_page(3)  # Show lid warning page
                    
                    # Wait for lid to close
                    while lid_sensor.value() == 1:
                        time.sleep(0.1)  # Small delay to prevent CPU hogging
                    
                    print("Lid is now closed - starting heating process")
                    time.sleep(0.5)  # Small delay for debounce
                
                # Determine which start button was pressed
                if page_id == 10 and component_id == 3:
                    print("Starting standard heating process")
                    start_heating_process()
                elif page_id == 9 and component_id == 9:
                    print("Starting alternative heating process")
                    start_heating_process2()
                    
            if page_id == 9:
                button_name = get_button_name(component_id)
                if button_name:
                    print(f"T/t button pressed: {button_name}")
                    
                    # Set the current edit context
                    current_edit_context = button_name
                    
                    # Set page title to indicate what's being edited
                    set_nextion_text("p4Title", f"Edit {current_edit_context}")
                    
                    # Switch to the keypad page
                    navigate_to_page(4)
                    
                    # Pre-fill with current value if it exists
                    if current_edit_context in edit_values:
                        current_value = str(edit_values[current_edit_context])
                        set_nextion_text("tValue", current_value)
                        print(f"Set initial value: {current_value}")
                        
                    # Handle the keypad entry
                    handle_keypad_entry()
            
            # Add other touch event handling as needed
            
        # Periodically update temperature display even when not heating
        if not heating_active and time.time() % 5 < 0.1:  # Every ~5 seconds
            temp = get_calibrated_temp()
            if temp is not None:
                send_nextion_command(f'n0.val={int(temp)}')
            
        time.sleep(0.1)  # Small delay to prevent CPU hogging

# Initialize the system
def initialize():
    """Initialize the system"""
    # Set initial values
    edit_values.update({
        "T0": 0, "t0": 0,
        "T1": 0, "t1": 0,
        "T2": 0, "t2": 0,
        "T3": 0, "t3": 0,
        "T4": 0, "t4": 0,
        "T5": 0, "t5": 0,
        "T6": 0, "t6": 0
    })
    
    # Initialize all text fields but don't change page
    for key, value in edit_values.items():
        set_nextion_text(key, str(value))
    
    print("System initialized with default values")
    
    
# Start the program
if __name__ == "__main__":
    print("Starting integrated PID slope control system")
    try:
        initialize()
        main_loop()
    except KeyboardInterrupt:
        print("Program stopped by user")
        output_pin.value(0)  # Ensure output is off on interrupt
        cooling_pin.value(0)  # Turn off cooling on interrupt
    except Exception as e:
        print(f"Error in main program: {e}")
        output_pin.value(0)  # Ensure output is off on error
        cooling_pin.value(0)  # Turn off cooling on error





