import time
import machine
from mlx90614 import MLX90614
from machine import Pin

# Initialize temperature sensor
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
sensor = MLX90614(i2c)

# Pulse control setup
INPUT_PIN = 2  # Using GP2 (Pin 4) to avoid I2C conflict
OUTPUT_PIN = 9  # GP9 (Pin 12)
input_pin = Pin(INPUT_PIN, Pin.IN, Pin.PULL_UP)
output_pin = Pin(OUTPUT_PIN, Pin.OUT)
output_pin.value(0)
pulse_detected = False  # Corrected variable name

def input_callback(pin):
    global pulse_detected
    pulse_detected = True

input_pin.irq(trigger=Pin.IRQ_RISING, handler=input_callback)

def generate_pulse():  # Fixed typo in function name
    output_pin.value(1)
    time.sleep_ms(1)
    output_pin.value(0)

# Calibration parameters
DELAY_VALUES = [4]
MEASUREMENT_DURATION = 100  # Reduced to 15 seconds per test
SAMPLE_INTERVAL = 1

def measure_slope(delay_ms):
    global pulse_detected  # Declare as global
    temps = []
    start_time = time.time()
    
    print(f"\nTesting delay: {delay_ms}ms")
    print("Time(s)\tTemp(°C)")
    
    while time.time() - start_time < MEASUREMENT_DURATION:
        if pulse_detected:  # Handle incoming pulses
            pulse_detected = False
            time.sleep_ms(delay_ms)
            generate_pulse()
        
        current_time = time.time() - start_time
        if len(temps) == 0 or current_time - temps[-1][0] >= SAMPLE_INTERVAL:
            try:
                temp = sensor.object_temp
                temps.append((current_time, temp))
                print(f"{current_time:.1f}\t{temp:.1f}")
            except Exception as e:
                print(f"Temp read error: {e}")
        
        time.sleep_ms(10)
    
    if len(temps) < 2:
        return 0.0
    
    slope = (temps[-1][1] - temps[0][1]) / (temps[-1][0] - temps[0][0])
    print(f"Delay {delay_ms}ms slope: {slope:.2f}°C/s")
    return slope

def run_calibration():
    results = {}
    print("Starting calibration...")
    
    for delay in DELAY_VALUES:
        slope = measure_slope(delay)
        results[delay] = slope
        print(f"Cooling down...")
        time.sleep(5)
    
    print("\n=== Results ===")
    print("Delay(ms)\tSlope(°C/s)")
    for delay, slope in sorted(results.items()):
        print(f"{delay}\t\t{slope:.2f}")
    
    return results

# Run calibration
try:
    calibration_data = run_calibration()
    print("Calibration complete!")
except Exception as e:
    print(f"Calibration failed: {e}")
