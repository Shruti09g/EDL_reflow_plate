import time
import machine
from mlx90614 import MLX90614

# Initialize I2C bus
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)

# Initialize UART for Nextion communication
uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))

# Scan for I2C devices
devices = i2c.scan()
print("I2C devices found:", [hex(device) for device in devices])

# Make sure MLX90614 is detected (usually at 0x5A)
sensor = MLX90614(i2c)

# Waveform settings
WAVEFORM_ID = 5        # Waveform component ID
PROFILE_CHANNEL = 0    # Channel for target profile (red)
MEASURED_CHANNEL = 1   # Channel for measured temp (green)
MAX_TEMP = 350         # Maximum temperature for scaling
START_TIME = time.time()

def send_nextion(cmd):
    uart.write(cmd + b'\xFF\xFF\xFF')
    time.sleep(0.1)

def setup_waveform():
    # Configure waveform display
    send_nextion(b'page 1')
    send_nextion(b'cle 5,0')        # Clear Channel 0 (profile)
    send_nextion(b'cle 5,1')        # Clear Channel 1 (measured)
    send_nextion(b's0.pco0=63488')  # Red for profile
    send_nextion(b's0.pco1=2016')   # Green for measured
    send_nextion(b's0.aph=0')       # Make waveform visible
    send_nextion(b'ref s0')

def scale_temp_to_value(temp):
    value = int((temp / MAX_TEMP) * 255)
    return min(255, max(0, value))

def get_profile_temp(elapsed):
    """Calculate temperature based on the profile and elapsed time"""
    if elapsed < 80:  # Phase 1: 0-80s, 25°C to 150°C
        return 25 + (150 - 25) * (elapsed / 80)
    elif elapsed < 175:  # Phase 2: 80-175s, hold at 150°C
        return 150
    elif elapsed < 240:  # Phase 3: 175-240s, 150°C to 220°C
        return 150 + (220 - 150) * ((elapsed - 175) / (240 - 175))
    elif elapsed < 250:  # Phase 4: 240-250s, hold at 220°C
        return 220
    else:  # Phase 5: >250s, 220°C to 100°C over 80s
        phase5_elapsed = elapsed - 250
        if phase5_elapsed < 80:
            return 220 - (220 - 100) * (phase5_elapsed / 80)
        else:  # After profile is complete, hold at 100°C
            return 100

def update_display(profile_temp, measured_temp):
    # Update numeric displays
    send_nextion(f'n0.val={int(measured_temp)}'.encode())  # Measured temp
    
    # Scale temperatures
    profile_value = scale_temp_to_value(profile_temp)
    measured_value = scale_temp_to_value(measured_temp)
    
    # Add points to waveforms
    send_nextion(f'add {WAVEFORM_ID},{PROFILE_CHANNEL},{profile_value}'.encode())
    send_nextion(f'add {WAVEFORM_ID},{MEASURED_CHANNEL},{measured_value}'.encode())
    send_nextion(b'ref s0')
    
    print(f"Profile: {profile_temp:.1f}°C | Measured: {measured_temp:.1f}°C")

# Initialize
print("Initializing Nextion display...")
setup_waveform()

# Main loop
try:
    while True:
        elapsed = time.time() - START_TIME
        
        # Get temperatures
        profile_temp = get_profile_temp(elapsed)
        measured_temp = sensor.object_temp
        
        # Update display
        update_display(profile_temp, measured_temp)
        time.sleep(1)  # Update every second

except KeyboardInterrupt:
    print("Stopped by user.")
