import time
import machine
from mlx90614 import MLX90614
from machine import Pin, Timer

# Initialize I2C bus for temperature sensor
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
sensor = MLX90614(i2c)

# Initialize UART for Nextion display
uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))

# Initialize pulse control system
INPUT_PIN = 2  # GP2 (Pin 4) for input pulse
OUTPUT_PIN = 9  # GP9 (Pin 12) for output pulse
input_pin = Pin(INPUT_PIN, Pin.IN, Pin.PULL_UP)
output_pin = Pin(OUTPUT_PIN, Pin.OUT)
output_pin.value(0)
pulse_detected = False

# Temperature profile parameters
PROFILE = {
    'preheat': {'start': 40, 'end': 120, 'time': 60, 'max_slope': 2.0},
    'soak': {'temp': 120, 'time': 45},
    'reflow': {'start': 120, 'end': 245, 'time': 30, 'max_slope': 4.0},
    'cooling': {'target_slope': -2.0}
}

# Nextion display functions
def send_nextion(cmd):
    uart.write(cmd + b'\xFF\xFF\xFF')
    time.sleep(0.01)

def setup_waveform():
    send_nextion(b'page 1')
    send_nextion(b's0.aph=0')
    send_nextion(b's0.pco0=63488')  # Red color
    send_nextion(b's0.gdc=0')
    send_nextion(b's0.vsc=1')
    send_nextion(b's0.hsc=1')
    send_nextion(b'cls 5,0')
    send_nextion(b'ref s0')

def update_nextion_temp(temp):
    send_nextion(f'n0.val={int(temp)}'.encode())
    send_nextion(f'add 5,0,{int(temp)}'.encode())
    send_nextion(b'ref s0')

# Pulse control functions
def input_callback(pin):
    global pulse_detected
    pulse_detected = True

def generate_pulse(timer):
    output_pin.value(1)
    time.sleep_ms(1)
    output_pin.value(0)

# Initialize system
input_pin.irq(trigger=Pin.IRQ_RISING, handler=input_callback)
timer = Timer()
prev_temp = None
current_delay = 3  # Initial delay in ms
current_state = 'preheat'
state_start_time = time.ticks_ms()

# Setup display
print("Initializing display...")
setup_waveform()
send_nextion(b'vis s0,1')
time.sleep(1)

def calculate_slope(current_temp, prev_temp, time_elapsed):
    if prev_temp is None or time_elapsed == 0:
        return 0
    return (current_temp - prev_temp) / (time_elapsed / 1000)  # °C/second

def update_state(current_temp, current_time):
    global current_state, state_start_time
    
    state_time = time.ticks_diff(current_time, state_start_time) / 1000  # in seconds
    
    if current_state == 'preheat':
        if current_temp >= PROFILE['preheat']['end'] or state_time > PROFILE['preheat']['time']:
            current_state = 'soak'
            state_start_time = current_time
    elif current_state == 'soak':
        if state_time > PROFILE['soak']['time']:
            current_state = 'reflow'
            state_start_time = current_time
    elif current_state == 'reflow':
        if current_temp >= PROFILE['reflow']['end'] or state_time > PROFILE['reflow']['time']:
            current_state = 'cooling'
            state_start_time = current_time

def get_target_slope():
    if current_state == 'preheat':
        required_temp_change = PROFILE['preheat']['end'] - PROFILE['preheat']['start']
        return required_temp_change / PROFILE['preheat']['time']
    elif current_state == 'soak':
        return 0  # Maintain temperature
    elif current_state == 'reflow':
        required_temp_change = PROFILE['reflow']['end'] - PROFILE['reflow']['start']
        return required_temp_change / PROFILE['reflow']['time']
    else:  # cooling
        return PROFILE['cooling']['target_slope']

def adjust_delay(current_slope, target_slope):
    global current_delay
    
    if target_slope == 0:  # Soak phase - maintain temperature
        current_delay = 20  # Minimal heating
    else:
        error = target_slope - current_slope
        if error > 0.5:  # Need more heating
            current_delay = max(1, current_delay - 1)
        elif error < -0.5:  # Need less heating
            current_delay = min(20, current_delay + 1)

# Main control loop
while True:
    current_time = time.ticks_ms()
    current_temp = sensor.object_temp
    
    # Calculate slope
    if prev_temp is not None:
        time_elapsed = time.ticks_diff(current_time, last_measurement_time)
        current_slope = calculate_slope(current_temp, prev_temp, time_elapsed)
    else:
        current_slope = 0
    
    # Update system state
    update_state(current_temp, current_time)
    target_slope = get_target_slope()
    
    # Adjust delay based on performance
    adjust_delay(current_slope, target_slope)
    
    # Handle pulses with current delay
    if pulse_detected:
        pulse_detected = False
        time.sleep_ms(current_delay)
        generate_pulse(timer)
    
    # Update display
    update_nextion_temp(current_temp)
    
    # Print debug info
    print(f"State: {current_state} | Temp: {current_temp:.1f}°C | "
          f"Slope: {current_slope:.2f}°C/s (Target: {target_slope:.2f}) | "
          f"Delay: {current_delay}ms")
    
    # Store values for next iteration
    prev_temp = current_temp
    last_measurement_time = current_time
    time.sleep(0.1)
