import requests
import json
import serial
import time
import logging
from typing import Optional

# ===========================
# Configuration Section
# ===========================

class Config:
    """Configuration parameters for the LoRa communication and API integration."""

    # ----- Serial Connection Parameters -----
    SERIAL_PORT = '/dev/serial0'    # Replace with your serial port (e.g., 'COM3' on Windows)
    BAUD_RATE = 57600               # Baud rate should match the LoRa module's settings
    TIMEOUT = 2                     # Timeout in seconds for serial read operations

    # ----- LoRa Module Commands -----
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
    COMMAND_RETRY_DELAY = 0.5      # Delay in seconds between retries

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

    # ----- API Integration -----
    POST_URL = 'https://fac7-171-96-191-39.ngrok-free.app/Dashboard/api/update_data.php'  # Replace with your actual URL
    SENSOR_ID = 123
    GATEWAY_ID = "GATEWAY001"

    # ----- Logging and Debugging -----
    DEBUG_MODE = True              # Set to True to enable debug logs
    LOG_LEVEL = logging.DEBUG if DEBUG_MODE else logging.INFO
    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"

# ===========================
# Logging Configuration
# ===========================

logging.basicConfig(level=Config.LOG_LEVEL, format=Config.LOG_FORMAT)
logger = logging.getLogger(__name__)

# ===========================
# Helper Functions
# ===========================

def string_to_hex(input_str: str) -> str:
    """
    Convert a regular string to a hex string.

    Args:
        input_str (str): The input string.

    Returns:
        str: The hexadecimal representation of the input string.
    """
    return ''.join(format(ord(c), '02X') for c in input_str)

def hex_to_string(hex_str: str) -> str:
    """
    Convert a hex string back to a regular string.

    Args:
        hex_str (str): The hexadecimal string.

    Returns:
        str: The converted string, or an empty string if conversion fails.
    """
    try:
        bytes_object = bytes.fromhex(hex_str)
        return bytes_object.decode('utf-8', errors='ignore')
    except ValueError:
        logger.error(f"Invalid hex string encountered: {hex_str}")
        return ""

# ===========================
# Class Definitions
# ===========================

class LoRaManager:
    """
    Manages LoRa module operations including configuration, sending commands, processing incoming messages,
    and sending data to a remote API via POST requests.
    """

    def __init__(self, config: Config):
        """
        Initialize the LoRaManager with the given configuration.

        Args:
            config (Config): Configuration parameters.
        """
        self.config = config
        self.ser: Optional[serial.Serial] = None

    def open_connection(self):
        """
        Opens the serial connection to the LoRa module.
        """
        try:
            self.ser = serial.Serial(
                port=self.config.SERIAL_PORT,
                baudrate=self.config.BAUD_RATE,
                timeout=self.config.TIMEOUT
            )
            logger.info(f"Opened serial port: {self.config.SERIAL_PORT} at {self.config.BAUD_RATE} baud.")
            # Flush any existing data in the buffers
            self.ser.flushInput()
            self.ser.flushOutput()
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.config.SERIAL_PORT}: {e}")
            raise

    def close_connection(self):
        """
        Closes the serial connection to the LoRa module.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial connection closed.")

    def send_command(self, command: str) -> str:
        """
        Send a command to the LoRa module and return the response.

        Args:
            command (str): The command string to send.

        Returns:
            str: The response from the LoRa module.
        """
        if not self.ser or not self.ser.is_open:
            logger.error("Serial connection is not open.")
            return ""

        full_command = f"{command}\r\n"
        try:
            self.ser.write(full_command.encode())
            logger.debug(f"Sent command: {command}")
            time.sleep(0.1)  # Small delay to allow processing
            response = self.ser.readline().decode('utf-8').strip()
            logger.debug(f"Received response: {response}")
            return response
        except serial.SerialException as e:
            logger.error(f"Serial exception during send_command: {e}")
            return ""
        except UnicodeDecodeError as e:
            logger.error(f"Unicode decode error: {e}")
            return ""

    def send_command_with_retry(self, command: str) -> bool:
        """
        Send a command with retries if acknowledgment is not received.

        Args:
            command (str): The command string to send.

        Returns:
            bool: True if the command was acknowledged, False otherwise.
        """
        for attempt in range(1, self.config.COMMAND_RETRIES + 1):
            response = self.send_command(command)
            if response.lower() == "ok":
                logger.debug(f"Command '{command}' acknowledged on attempt {attempt}.")
                return True
            else:
                logger.warning(f"Attempt {attempt} failed for command '{command}': {response}")
                time.sleep(self.config.COMMAND_RETRY_DELAY)
        logger.error(f"All {self.config.COMMAND_RETRIES} attempts failed for command '{command}'.")
        return False

    def configure_gateway(self):
        """
        Configure the LoRa module to act as a gateway (receiver).
        """
        try:
            # Reset the module
            logger.info("Resetting LoRa module...")
            self.send_command(self.config.LORA_RESET_COMMAND)
            time.sleep(2)  # Wait for reset

            # Pause MAC operations
            logger.info("Pausing MAC operations...")
            self.send_command(self.config.LORA_MAC_PAUSE_COMMAND)
            time.sleep(0.5)

            # Set radio parameters with retries
            if not self.send_command_with_retry(self.config.LORA_RADIO_BW_COMMAND):
                raise Exception("Failed to set bandwidth.")
            if not self.send_command_with_retry(self.config.LORA_RADIO_CR_COMMAND):
                raise Exception("Failed to set coding rate.")
            if not self.send_command_with_retry(self.config.LORA_RADIO_PWR_COMMAND):
                raise Exception("Failed to set power.")

            # Set frequency
            logger.info("Setting frequency to 910 MHz...")
            if self.send_command_with_retry(self.config.LORA_RADIO_FREQ_COMMAND):
                logger.info("Frequency set successfully.")
            else:
                logger.error("Failed to set frequency.")

            # Set spreading factor (SF7)
            logger.info("Setting spreading factor to SF7...")
            if self.send_command_with_retry(self.config.LORA_RADIO_SF_COMMAND):
                logger.info("Spreading factor set successfully.")
            else:
                logger.error("Failed to set spreading factor.")

            # Put the module into receive mode indefinitely (0 means no timeout)
            logger.info("Putting LoRa module into receive mode...")
            if self.send_command_with_retry(self.config.LORA_RADIO_RX_COMMAND):
                logger.info("LoRa module is now in receive mode.")
            else:
                logger.error("Failed to set receive mode.")

        except Exception as e:
            logger.error(f"Error configuring gateway: {e}")

    def get_system_version(self):
        """
        Request the system version from the LoRa module.
        """
        logger.info("Requesting system version from LoRa module...")
        response = self.send_command(self.config.LORA_SYS_GET_VER_COMMAND)
        if response:
            logger.info(f"LoRa Module Version: {response}")
        else:
            logger.warning("No response received for system version.")

    def send_post_request(self, data: dict):
        """
        Send a POST request with the given data to the configured API endpoint.

        Args:
            data (dict): The data to send in the POST request.
        """
        headers = {'Content-Type': 'application/json'}
        try:
            response = requests.post(self.config.POST_URL, data=json.dumps(data), headers=headers)
            if response.status_code == 200:
                try:
                    response_data = response.json()
                    logger.info(f"POST request successful. Response: {response_data}")
                except json.JSONDecodeError:
                    logger.warning("POST request successful but failed to decode JSON response.")
            else:
                logger.error(f"Failed to send POST request. Status code: {response.status_code}, Response: {response.text}")
        except requests.RequestException as e:
            logger.error(f"Exception occurred during POST request: {e}")

    def process_received_message(self, message: str):
        """
        Process and convert received hex messages to readable JSON data, then send it via POST.

        Args:
            message (str): The raw message received from LoRa.
        """
        if message.startswith("radio_rx"):
            # Example message format: radio_rx <hex_data>
            parts = message.split()
            if len(parts) >= 2:
                hex_data = parts[1]
                readable_message = hex_to_string(hex_data)
                logger.debug(f"Received Packet: {readable_message}")

                # Attempt to parse JSON
                try:
                    data = json.loads(readable_message)
                    logger.info("Parsed JSON Data:")
                    for key, value in data.items():
                        logger.info(f"  {key}: {value}")

                    # Prepare data for POST request
                    post_data = {
                        "sensor_id": self.config.SENSOR_ID,
                        "gateway_id": self.config.GATEWAY_ID,
                        "data_kwh" : data.get("TotalActivePower", 0.0)
                    }

                    # Send POST request with the data
                    self.send_post_request(post_data)

                except json.JSONDecodeError:
                    logger.error(f"Failed to decode JSON from message: {readable_message}")
                except Exception:
                    pass
            else:
                logger.error(f"Unexpected radio_rx format: {message}")

        elif message.startswith("radio_err"):
            # Handle receive errors
            logger.error(f"Receive Error: {message}")
            logger.info("Attempting to reset and reconfigure LoRa module...")
            self.configure_gateway()

        elif message.startswith("ok"):
            # Acknowledgment for commands
            logger.debug(f"Received acknowledgment: {message}")
            # Can be used to confirm successful commands

        else:
            # Handle other responses or unexpected messages
            logger.warning(f"Unexpected response: {message}")

    def listen_for_messages(self):
        """
        Continuously listen for incoming messages and process them.
        """
        logger.info("Listening for incoming messages...")
        try:
            while True:
                if self.ser.in_waiting > 0:
                    raw_message = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw_message:
                        logger.debug(f"Raw Message Received: {raw_message}")
                        self.process_received_message(raw_message)

                        # Re-issue receive command to ensure continuous listening
                        if not self.send_command_with_retry(self.config.LORA_RADIO_RX_COMMAND):
                            logger.error("Failed to re-issue receive command.")

                time.sleep(0.1)  # Small delay to prevent CPU overuse
        except KeyboardInterrupt:
            logger.info("Stopping receiver...")
        except Exception as e:
            logger.error(f"Error while listening: {e}")

# ===========================
# Main Execution
# ===========================

def main():
    """
    Main function to initialize and run the LoRa communication.
    """
    config = Config()
    lora_manager = LoRaManager(config)

    try:
        lora_manager.open_connection()
        lora_manager.configure_gateway()
        lora_manager.get_system_version()

        # Start listening for incoming messages
        lora_manager.listen_for_messages()

    except serial.SerialException as e:
        logger.error(f"Serial Exception: {e}")
    except Exception as e:
        logger.error(f"Unexpected Exception: {e}")
    finally:
        lora_manager.close_connection()

if __name__ == "__main__":
    main()
