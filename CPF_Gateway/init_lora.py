import serial
import time
import RPi.GPIO as GPIO

# Configure GPIO
LORA_ENABLE_PIN = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(LORA_ENABLE_PIN, GPIO.OUT)

# Enable the LoRa module
GPIO.output(LORA_ENABLE_PIN, GPIO.HIGH)
print("LoRa module enabled")
time.sleep(1)  # Allow some time for the module to initialize

# Set up UART connection
SERIAL_PORT = "/dev/serial0"  # Change if using a different port
BAUD_RATE = 57600

try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as lora_serial:
        print("Serial connection established")
        
        # Send a test command or message
        test_message = "sys get ver\r\n"  # Replace with a valid LoRa command if needed
        lora_serial.write(test_message.encode('utf-8'))
        print(f"Sent: {test_message.strip()}")
        
        # Wait for a response
        time.sleep(0.5)
        if lora_serial.in_waiting > 0:
            response = lora_serial.read(lora_serial.in_waiting).decode('utf-8')
            print(f"Received: {response.strip()}")
        else:
            print("No response from LoRa module")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Disable the LoRa module
    GPIO.output(LORA_ENABLE_PIN, GPIO.LOW)
    print("LoRa module disabled")
    GPIO.cleanup()

