import serial
import time
import json

# ===========================
# Configuration Section
# ===========================

# ----- Serial Connection Parameters -----

SERIAL_PORT = '/dev/serial0'    # Replace with your serial port (e.g., 'COM3' on Windows)
BAUD_RATE = 57600               # Baud rate should match the LoRa module's settings
TIMEOUT = 2                     # Timeout in seconds for serial read operations

# ----- LoRa Module Settings -----

LORA_RESET_COMMAND = "sys reset"
LORA_MAC_PAUSE_COMMAND = "mac pause"
LORA_RADIO_BW_COMMAND = "radio set bw 125"
LORA_RADIO_CR_COMMAND = "radio set cr 4/5"
LORA_RADIO_PWR_COMMAND = "radio set pwr 20"
LORA_RADIO_FREQ_COMMAND = "radio set freq 910000000"
LORA_RADIO_SF_COMMAND = "radio set sf sf7"
LORA_RADIO_RX_COMMAND = "radio rx 0"
LORA_SYS_GET_VER_COMMAND = "sys get ver"
LORA_RADIO_TX_PREFIX = "radio tx "

# ----- Command Retry Settings -----

COMMAND_RETRIES = 3            # Number of retries for sending commands
COMMAND_RETRY_DELAY = 0.5       # Delay in seconds between retries

# ----- Message Processing -----

JSON_KEYS = {
    "Va": "Va",
    "Vb": "Vb",
    "Vc": "Vc",
    "Ia": "Ia",
    "Ib": "Ib",
    "Ic": "Ic",
    "ActivePower": "ActivePower",
    "TotalActivePower": "TotalActivePower"
}

# ----- Logging and Debugging -----

DEBUG_MODE = True              # Set to True to enable debug prints

# ===========================
# Helper Functions
# ===========================

def log_debug(message):
    """
    Log debug messages if DEBUG_MODE is enabled.
    """
    if DEBUG_MODE:
        print(message)

def string_to_hex(input_str):
    """
    Convert a regular string to a hex string.
    """
    return ''.join(format(ord(c), '02X') for c in input_str)

def hex_to_string(hex_str):
    """
    Convert a hex string back to a regular string.
    """
    try:
        bytes_object = bytes.fromhex(hex_str)
        return bytes_object.decode('utf-8', errors='ignore')
    except ValueError:
        log_debug(f"Invalid hex string encountered: {hex_str}")
        return ""

def send_command(ser, command):
    """
    Send a command to the LoRa module and return the response.
    """
    ser.write((command + "\r\n").encode())
    time.sleep(0.1)  # Small delay to allow processing
    response = ser.readline()
    try:
        return response.decode('utf-8').strip()
    except UnicodeDecodeError:
        log_debug(f"Non-text response received: {response}")
        return ""

def send_command_with_retry(ser, command, retries=COMMAND_RETRIES, delay_sec=COMMAND_RETRY_DELAY):
    """
    Send a command with retries if acknowledgment is not received.
    """
    for attempt in range(retries):
        response = send_command(ser, command)
        if response.lower() == "ok":
            log_debug(f"Command '{command}' acknowledged.")
            return True
        else:
            log_debug(f"Attempt {attempt + 1} failed for command '{command}': {response}")
            time.sleep(delay_sec)
    log_debug(f"All {retries} attempts failed for command '{command}'")
    return False

def configure_gateway(ser):
    """
    Configure the LoRa module to act as a gateway (receiver).
    """
    try:
        # Reset the module
        log_debug("Resetting LoRa module...")
        send_command(ser, LORA_RESET_COMMAND)
        time.sleep(2)  # Wait for reset

        # Pause MAC operations
        send_command(ser, LORA_MAC_PAUSE_COMMAND)
        time.sleep(0.5)

        # Set radio parameters with retries
        if not send_command_with_retry(ser, LORA_RADIO_BW_COMMAND):
            raise Exception("Failed to set bandwidth")

        if not send_command_with_retry(ser, LORA_RADIO_CR_COMMAND):
            raise Exception("Failed to set coding rate")

        if not send_command_with_retry(ser, LORA_RADIO_PWR_COMMAND):
            raise Exception("Failed to set power")

        # Set frequency
        log_debug("Setting frequency to 910 MHz...")
        if send_command_with_retry(ser, LORA_RADIO_FREQ_COMMAND):
            log_debug("Frequency set successfully.")
        else:
            log_debug("Failed to set frequency.")

        # Set spreading factor (SF7)
        log_debug("Setting spreading factor to SF7...")
        if send_command_with_retry(ser, LORA_RADIO_SF_COMMAND):
            log_debug("Spreading factor set successfully.")
        else:
            log_debug("Failed to set spreading factor.")

        # Put the module into receive mode indefinitely (0 means no timeout)
        log_debug("Putting LoRa module into receive mode...")
        if send_command_with_retry(ser, LORA_RADIO_RX_COMMAND):
            log_debug("LoRa module is now in receive mode.")
        else:
            log_debug("Failed to set receive mode.")

    except Exception as e:
        log_debug(f"Error configuring gateway: {e}")

def process_received_message(message, ser):
    """
    Process and convert received hex messages to readable JSON data.
    """
    if message.startswith("radio_rx"):
        # Example message format: radio_rx <hex_data>
        parts = message.split()
        if len(parts) >= 2:
            hex_data = parts[1]
            readable_message = hex_to_string(hex_data)
            log_debug(f"Received Packet: {readable_message}")

            # Attempt to parse JSON
            try:
                data = json.loads(readable_message)
                log_debug("Parsed JSON Data:")
                for key, value in data.items():
                    log_debug(f"  {key}: {value}")
                # Example of handling data
                total_active_power = data.get(JSON_KEYS["TotalActivePower"])
                log_debug(f"Total Active Power: {total_active_power}")
            except json.JSONDecodeError:
                log_debug(f"Failed to decode JSON from message: {readable_message}")
        else:
            log_debug(f"Unexpected radio_rx format: {message}")

    elif message.startswith("radio_err"):
        # Handle receive errors
        log_debug(f"Receive Error: {message}")
        log_debug("Attempting to reset and reconfigure LoRa module...")
        configure_gateway(ser)

    elif message.startswith("ok"):
        # Acknowledgment for commands
        log_debug(f"Received acknowledgment: {message}")
        pass  # Can be ignored or logged if necessary

    else:
        # Handle other responses or unexpected messages
        log_debug(f"Unexpected response: {message}")

def listen_for_messages(ser):
    """
    Continuously listen for incoming messages and process them.
    """
    log_debug("Listening for incoming messages...")
    try:
        while True:
            if ser.in_waiting > 0:
                # Read the incoming message
                raw_message = ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_message:
                    log_debug(f"Raw Message Received: {raw_message}")
                    process_received_message(raw_message, ser)

                # Re-issue receive command to ensure continuous listening
                send_command_with_retry(ser, LORA_RADIO_RX_COMMAND)
            time.sleep(0.1)  # Small delay to prevent CPU overuse
    except KeyboardInterrupt:
        log_debug("Stopping receiver...")
    except Exception as e:
        log_debug(f"Error while listening: {e}")

# ===========================
# Main Execution
# ===========================

if __name__ == "__main__":
    try:
        # Initialize serial connection
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT
        )
        log_debug(f"Opened serial port: {SERIAL_PORT} at {BAUD_RATE} baud.")

        # Flush any existing data in the buffers
        ser.flushInput()
        ser.flushOutput()

        # Configure the LoRa module as a gateway (receiver)
        configure_gateway(ser)

        # Start listening for incoming messages
        listen_for_messages(ser)

    except serial.SerialException as e:
        log_debug(f"Serial Exception: {e}")
    except Exception as e:
        log_debug(f"Unexpected Exception: {e}")
