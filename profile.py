import time
import machine
from mlx90614 import MLX90614

# Initialize I2C bus
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)

# Initialize UART for Nextion communication
uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))

# Scan for I2C devices
devices = i2c.scan()

if devices:
    print("I2C devices found:", [hex(device) for device in devices])
else:
    print("No I2C devices found")

# Initialize the MLX90614 sensor
sensor = MLX90614(i2c)
# Function to send command to Nextion display
def send_nextion(cmd):
    uart.write(cmd + b'\xFF\xFF\xFF')  # Nextion commands end with 3xFF
    time.sleep(0.01)

def setup_waveform():
   # Reset and configure waveform
    send_nextion(b'page 1')
    send_nextion(b's0.aph=0')  # Make waveform visible
    send_nextion(b's0.pco0=63488')  # Red line color
    send_nextion(b's0.gdc=0')  # No grid for cleaner look
    send_nextion(b's0.vsc=1')  # Enable vertical scroll
    send_nextion(b's0.hsc=1')  # Enable horizontal scroll
    send_nextion(b'cls 5,0')  # Clear channel 0 data
    send_nextion(b'ref s0')  # Refresh
    
# Function to update temperature on Nextion display
def update_nextion_temp(temp):
    # Update number display (n0 with id 3)
    send_nextion(f'n0.val={int(temp)}'.encode())
    
    # Add to waveform (s0 with id 5)
    send_nextion(f'add 5,0,{int(temp)}'.encode())
    
    # Auto-refresh waveform
    send_nextion(b'ref s0')
    
# Variables to store previous temperature readings
prev_object_temp = None
prev_ambient_temp = None

# Initialize display
print("Initializing display...")
setup_waveform()
send_nextion(b'vis s0,1')  # Ensure waveform is visible
time.sleep(1)  # Important initial delay


while True:
    # Read current temperatures
    ambient_temp = sensor.ambient_temp  # Access ambient temperature directly
    object_temp = sensor.object_temp    # Access object temperature directly
    
    # Calculate slope (difference between current and previous readings)
    if prev_object_temp is not None and prev_ambient_temp is not None:
        object_slope = object_temp - prev_object_temp
        ambient_slope = ambient_temp - prev_ambient_temp
    else:
        object_slope = 0  # No previous reading, slope is 0
        ambient_slope = 0
    
    # Print temperatures and slopes
    
    print(f"Temperature: {object_temp:.2f}°C")
    print(f"Slope: {object_slope:.2f}°C/s")
    print("-" * 20)  # Separator for readability
    
    # Update Nextion display with current temperature
    update_nextion_temp(object_temp)
    
    # Update previous temperature readings
    prev_object_temp = object_temp
    prev_ambient_temp = ambient_temp
    
    time.sleep(1)  # Wait for 1 second
