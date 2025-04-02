import time
import machine
from mlx90614 import MLX90614

# Initialize I2C bus
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)

# Initialize UART for Nextion communication
uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))
sensor = MLX90614(i2c)

# Waveform settings
WAVEFORM_ID = 5  # Change to your waveform's ID
CHANNEL = 0      # Use Channel 0
MAX_TEMP = 250   # Set your expected maximum temperature (°C)

def send_nextion(cmd):
    uart.write(cmd + b'\xFF\xFF\xFF')
    time.sleep(0.1)

def setup_waveform():
    # Basic waveform configuration
    send_nextion(b'page 1')
    send_nextion(b's0.aph=0')       # Make waveform visible
    send_nextion(b's0.pco0=63488')   # Green color for Channel 0
    send_nextion(b'cle 5,0')        # Clear Channel 0 data
    send_nextion(b'ref s0')

def scale_temp_to_value(temp):
    # Scale temperature to 0-255 range, with adjustable max temp
    value = int((temp / MAX_TEMP) * 255)
    return min(255, max(0, value))  # Clamp to 0-255

def update_nextion_temp(temp):
    # Update number display (n0 with id 3)
    send_nextion(f'n0.val={int(temp)}'.encode())
    value = scale_temp_to_value(temp)
    print(f"Temp: {temp:.1f}°C → Value: {value}")
    send_nextion(f'add {WAVEFORM_ID},{CHANNEL},{value}'.encode())
    send_nextion(b'ref s0')

# Initialize
print("Initializing Nextion display...")
setup_waveform()

# Main loop
try:
    while True:
        temp = sensor.object_temp
        
        # Optional: Add some artificial variation for testing
        # temp += (time.time() % 10) - 5  # Add ±5°C variation
        
        update_nextion_temp(temp)
        time.sleep(1)  # Update every second

except KeyboardInterrupt:
    print("Stopped by user.")
