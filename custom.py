from machine import UART, Pin
import time

# Initialize UART for Nextion display
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Global variables
current_edit_context = ""  # Stores which value is being edited (e.g., "t1", "T2")
edit_values = {}  # Dictionary to store the values

# Command terminator for Nextion
CMD_END = b'\xff\xff\xff'

def send_nextion_command(command):
    """Send a command to the Nextion display"""
    print(f"Sending command: {command}")
    uart.write(command.encode())
    uart.write(CMD_END)
    time.sleep(0.1)  # Give display time to process

def set_nextion_text(component_id, text):
    """Set text of a component on Nextion display"""
    command = f'{component_id}.txt="{text}"'
    send_nextion_command(command)
        
def navigate_to_page(page_num):
    """Navigate to a specific page on the Nextion display"""
    print(f"Navigating to page: {page_num}")
    send_nextion_command(f'page {page_num}')
    
    # If we're going to the main page, update all values quickly
    if page_num == 9:
        # No delay here - send commands immediately
        update_all_display_values(fast=True)
        
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
    """Map component IDs to button names based on your Nextion design"""
    # You'll need to adjust this mapping based on your actual component IDs in Nextion
    # These are just examples - replace with your actual component IDs
    t_buttons = {2: "T0", 3: "T1", 4: "T2", 5: "T3", 6: "T4", 7: "T5", 8: "T6"}
    T_buttons = {11: "t0", 12: "t1", 13: "t2", 14: "t3", 15: "t4", 16: "t5", 17: "t6"}
    
    # Combine the dictionaries
    all_buttons = {}
    all_buttons.update(t_buttons)
    all_buttons.update(T_buttons)
    
    return all_buttons.get(component_id, None)

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

def main_loop():
    """Main program loop"""
    global current_edit_context
    while True:
        touch_event = read_nextion_touch()
        if touch_event:
            page_id, component_id, action_type = touch_event
            
            # Check if on page 9 and a button was touched
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
    print("Starting Nextion control program")
    initialize()
    main_loop()
