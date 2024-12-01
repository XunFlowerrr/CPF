/**
 * @file main.cpp
 * @brief ESP32-based system integrating LoRa (RN2903), OLED display, and Modbus voltage meter.
 *
 * This project sets up an ESP32 microcontroller to communicate with an RN2903 LoRa module,
 * an OLED display, and a Modbus-compatible voltage meter. It reads sensor data via Modbus,
 * displays it on the OLED, and transmits the data over LoRa in hexadecimal-encoded JSON format.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>

// ===========================
// Configuration Section
// ===========================

/**
 * @namespace Pins
 * @brief Defines the pin configurations for various modules.
 */
namespace Pins
{
  // LoRa Module Pins
  constexpr uint8_t LORA_RX = 16;  /**< RN2903 TX → ESP32 RX (UART2 RX) */
  constexpr uint8_t LORA_TX = 17;  /**< RN2903 RX → ESP32 TX (UART2 TX) */
  constexpr uint8_t LORA_RST = 5;  /**< RN2903 RESET */
  constexpr uint8_t LORA_DIO0 = 4; /**< RN2903 DIO0 (Optional) */
  constexpr uint8_t POWER = 33;    /**< Power supply to RN2903 */

  // OLED Display Pins
  constexpr uint8_t OLED_SDA = 21; /**< OLED SDA */
  constexpr uint8_t OLED_SCL = 22; /**< OLED SCL */

  // Modbus Volt Meter Pins
  constexpr uint8_t MODBUS_RX = 12; /**< Voltmeter TX → ESP32 RX (UART1 RX) */
  constexpr uint8_t MODBUS_TX = 18; /**< Voltmeter RX → ESP32 TX (UART1 TX) */
}

/**
 * @namespace UserSerialConfig
 * @brief Configures serial communication parameters.
 */
namespace UserSerialConfig
{
  constexpr unsigned long BAUD_RATE = 115200; /**< Serial Monitor baud rate */

  // UART Ports
  constexpr int MODBUS_PORT = 1; /**< UART1 for Modbus */
  constexpr int LORA_PORT = 2;   /**< UART2 for LoRa */
}

/**
 * @namespace OLEDConfig
 * @brief Configuration settings for the OLED display.
 */
namespace OLEDConfig
{
  constexpr int WIDTH = 128;                     /**< OLED display width in pixels */
  constexpr int HEIGHT = 64;                     /**< OLED display height in pixels */
  constexpr int TEXT_SIZE = 1;                   /**< Text size for OLED */
  constexpr uint16_t TEXT_COLOR = SSD1306_WHITE; /**< Text color for OLED */
  constexpr unsigned long WELCOME_DELAY = 2000;  /**< Delay to display welcome message (ms) */
}

/**
 * @namespace ModbusRegisters
 * @brief Defines Modbus register addresses for voltage, current, and power measurements.
 */
namespace ModbusRegisters
{
  // Voltage Registers
  constexpr uint16_t VA = 0; /**< 40001 - VA */
  constexpr uint16_t VB = 1; /**< 40002 - VB */
  constexpr uint16_t VC = 2; /**< 40003 - VC */

  // Current Registers
  constexpr uint16_t IA = 3; /**< 40004 - IA (UINT32) */
  constexpr uint16_t IB = 5; /**< 40006 - IB (UINT32) */
  constexpr uint16_t IC = 7; /**< 40008 - IC (UINT32) */

  // Power Registers
  constexpr uint16_t TOTAL_APPARENT_POWER = 11; /**< 40012 - Total Apparent Power (LUINT32) */
  constexpr uint16_t POWER_FACTOR = 10;         /**< 40011 - Power Factor (INT16) */
  constexpr uint16_t TOTAL_ACTIVE_POWER = 23;   /**< 40024 - Total Active Power (0.01 W) */
}

/**
 * @brief Configuration toggle to use mock data instead of actual Modbus data.
 */
constexpr bool USE_MOCK_DATA = true; /**< Set to true to use mock data */

/**
 * @namespace Delays
 * @brief Defines various delay durations in milliseconds.
 */
namespace Delays
{
  constexpr unsigned long SHORT = 100;       /**< Short delay */
  constexpr unsigned long MEDIUM = 500;      /**< Medium delay */
  constexpr unsigned long LONG = 1000;       /**< Long delay */
  constexpr unsigned long EXTRA_LONG = 2000; /**< Extra long delay */
}

/**
 * @namespace LoRaCommands
 * @brief Defines command strings for configuring the LoRa module.
 */
namespace LoRaCommands
{
  const char *MAC_PAUSE = "mac pause";                 /**< Pause MAC operations */
  const char *RADIO_BW = "radio set bw 125";           /**< Set radio bandwidth to 125 kHz */
  const char *RADIO_CR = "radio set cr 4/5";           /**< Set radio coding rate to 4/5 */
  const char *RADIO_PWR = "radio set pwr 20";          /**< Set radio power to 20 dBm */
  const char *RADIO_FREQ = "radio set freq 910000000"; /**< Set radio frequency to 910 MHz */
  const char *RADIO_SF = "radio set sf sf7";           /**< Set spreading factor to SF7 */
  const char *SYS_GET_VER = "sys get ver";             /**< Get system version */
  const char *SYS_RESET = "sys reset";                 /**< Reset system */
  const char *RADIO_TX_PREFIX = "radio tx ";           /**< Prefix for transmitting data */
}

/**
 * @namespace JSONKeys
 * @brief Defines JSON keys used in data transmission.
 */
namespace JSONKeys
{
  const char *VA = "Va";                               /**< Voltage A */
  const char *VB = "Vb";                               /**< Voltage B */
  const char *VC = "Vc";                               /**< Voltage C */
  const char *IA = "Ia";                               /**< Current A */
  const char *IB = "Ib";                               /**< Current B */
  const char *IC = "Ic";                               /**< Current C */
  const char *ACTIVE_POWER = "ActivePower";            /**< Active Power */
  const char *TOTAL_ACTIVE_POWER = "TotalActivePower"; /**< Total Active Power */
}

// ===========================
// Global Objects and Variables
// ===========================

/**
 * @brief OLED Display Instance using Adafruit_SSD1306 library.
 */
Adafruit_SSD1306 display(OLEDConfig::WIDTH, OLEDConfig::HEIGHT, &Wire, -1);

/**
 * @brief ModbusMaster Instance for Modbus communication.
 */
ModbusMaster modbus;

/**
 * @brief HardwareSerial instances for Modbus and LoRa communication.
 */
HardwareSerial modbusSerial(UserSerialConfig::MODBUS_PORT); /**< Serial for Modbus */
HardwareSerial loraSerial(UserSerialConfig::LORA_PORT);     /**< Serial for LoRa */

/**
 * @brief Stores the latest data received from LoRa.
 */
String latestLoraData = "";

/**
 * @brief Timing variables for loop interval management.
 */
unsigned long previousMillis = 0;
constexpr unsigned long LOOP_INTERVAL = Delays::EXTRA_LONG;

// ===========================
// Utility Functions
// ===========================

/**
 * @brief Converts a string to its hexadecimal representation.
 *
 * @param input The input string to convert.
 * @return String The hexadecimal representation of the input string.
 */
String stringToHex(const String &input)
{
  String hexString = "";
  for (unsigned int i = 0; i < input.length(); i++)
  {
    char hex[3];
    sprintf(hex, "%02X", input[i]);
    hexString += String(hex);
  }
  return hexString;
}

/**
 * @brief Converts a hexadecimal string back to a regular string.
 *
 * @param hex The hexadecimal string to convert.
 * @return String The converted regular string.
 */
String hexToString(const String &hex)
{
  String output = "";
  for (unsigned int i = 0; i < hex.length(); i += 2)
  {
    String part = hex.substring(i, i + 2);
    char chr = (char)strtol(part.c_str(), NULL, 16);
    output += chr;
  }
  return output;
}

// ===========================
// Function Prototypes
// ===========================
void resetLoRaModule(); // Added declaration before class definitions
void initializeOLED();
void displayData(const String &jsonData, float activePowerKW, float totalActivePower);

// ===========================
// Class Definitions
// ===========================

/**
 * @class LoRaManager
 * @brief Manages LoRa module operations including initialization, sending, and receiving data.
 */
class LoRaManager
{
public:
  /**
   * @brief Initializes LoRa serial communication and configures the LoRa module.
   */
  void begin()
  {
    Serial.println("Initializing LoRa Serial Communication...");
    delay(Delays::SHORT);
    Serial.println("LoRa Serial Communication initialized at 57600 bps.");

    // Send initialization commands
    initializeModule();

    // Ensure LoRa module responds to 'sys get ver'
    bool versionReceived = false;
    while (!versionReceived)
    {
      Serial.println("Sending 'sys get ver' to LoRa module...");
      sendCommand(LoRaCommands::SYS_GET_VER, Delays::LONG);

      // Wait for response within timeout
      unsigned long startTime = millis();
      unsigned long timeout = 5000; // 5 seconds timeout
      while (millis() - startTime < timeout)
      {
        if (receiveData(latestLoraData))
        {
          // Check if the response contains version information
          if (latestLoraData.startsWith("RN2903") || latestLoraData.startsWith("RN2"))
          {
            Serial.println("LoRa module version received: " + latestLoraData);
            versionReceived = true;
            break;
          }
        }
        delay(100); // Small delay to avoid tight loop
      }

      if (!versionReceived)
      {
        Serial.println("No response to 'sys get ver'. Resetting LoRa module...");
        resetLoRaModule();
        initializeModule(); // Reinitialize module after reset
      }
    }

    Serial.println("LoRa module is ready and responding.");
  }

  /**
   * @brief Sends a series of commands to configure the LoRa module.
   */
  void initializeModule()
  {
    Serial.println("Initializing LoRa Module with custom settings...");

    sendCommand(LoRaCommands::MAC_PAUSE, Delays::MEDIUM);
    sendCommand(LoRaCommands::RADIO_BW, Delays::MEDIUM);
    sendCommand(LoRaCommands::RADIO_CR, Delays::MEDIUM);
    sendCommand(LoRaCommands::RADIO_PWR, Delays::MEDIUM);
    sendCommand(LoRaCommands::RADIO_FREQ, Delays::MEDIUM);
    sendCommand(LoRaCommands::RADIO_SF, Delays::MEDIUM);
  }

  /**
   * @brief Sends a command string to the LoRa module.
   *
   * @param cmd The command string to send.
   * @param delayTime The delay after sending the command.
   */
  void sendCommand(const char *cmd, unsigned long delayTime)
  {
    loraSerial.println(cmd);
    Serial.print("Sent to LoRa: ");
    Serial.println(cmd);
    delay(delayTime);
  }

  /**
   * @brief Sends hexadecimal-encoded data via LoRa.
   *
   * @param dataHex The data string in hexadecimal format.
   */
  void sendData(const String &dataHex)
  {
    String command = String(LoRaCommands::RADIO_TX_PREFIX) + dataHex;
    loraSerial.println(command);
    Serial.println("Sending via LoRa: " + command);
  }

  /**
   * @brief Receives data from the LoRa module if available.
   *
   * @param data Reference to a string where received data will be stored.
   * @return true If data was received successfully.
   * @return false If no data was received.
   */
  bool receiveData(String &data)
  {
    if (loraSerial.available())
    {
      data = loraSerial.readStringUntil('\n');
      data.trim();
      Serial.print("Received from LoRa: ");
      Serial.println(data);
      return true;
    }
    return false;
  }
};

/**
 * @class ModbusManager
 * @brief Manages Modbus communication and data retrieval from the voltage meter.
 */
class ModbusManager
{
public:
  /**
   * @brief Initializes Modbus serial communication.
   */
  void begin()
  {
    Serial.println("Initializing Modbus Communication...");
    modbusSerial.begin(9600, SERIAL_8N1, Pins::MODBUS_RX, Pins::MODBUS_TX);
    delay(Delays::SHORT);
    Serial.println("Modbus Serial Communication initialized at 9600 bps.");

    modbus.begin(1, modbusSerial); // Slave ID 1
  }

  /**
   * @brief Reads a set of holding registers from the Modbus device.
   *
   * @param start The starting register address.
   * @param count The number of registers to read.
   * @param buffer Pointer to an array where the read data will be stored.
   * @return true If the registers were read successfully.
   * @return false If there was an error reading the registers.
   */
  bool readRegisters(uint16_t start, uint16_t count, uint16_t *buffer)
  {
    uint8_t result = modbus.readHoldingRegisters(start, count);
    if (result == modbus.ku8MBSuccess)
    {
      for (uint16_t i = 0; i < count; i++)
      {
        buffer[i] = modbus.getResponseBuffer(i);
      }
      return true;
    }
    else
    {
      Serial.print("Modbus Read Error (Start: ");
      Serial.print(start);
      Serial.print("): ");
      Serial.println(result, HEX);
      return false;
    }
  }
};

// ===========================
// Function Prototypes
// ===========================
void resetLoRaModule();
void initializeOLED();
void displayData(const String &jsonData, float activePowerKW, float totalActivePower);

// ===========================
// Global Instances
// ===========================
LoRaManager loraManager;     /**< Instance of LoRaManager */
ModbusManager modbusManager; /**< Instance of ModbusManager */

// ===========================
// Setup Function
// ===========================

/**
 * @brief Arduino setup function. Initializes serial communication, power and reset pins for LoRa,
 *        OLED display, LoRa and Modbus communication, and sends initial commands.
 */
void setup()
{
  // Initialize Serial Monitor
  Serial.begin(UserSerialConfig::BAUD_RATE);
  loraSerial.begin(57600, SERIAL_8N1, Pins::LORA_RX, Pins::LORA_TX);
  // while (!Serial)
  // {
  //   ; // Wait for Serial Monitor to open
  // }
  Serial.println("==================================");
  Serial.println("ESP32, RN2903 LoRa, and Modbus Setup");
  Serial.println("==================================");

  // Initialize Power Pin for LoRa Module
  pinMode(Pins::POWER, OUTPUT);
  digitalWrite(Pins::POWER, HIGH); // Turn on RN2903
  Serial.println("Power to RN2903 enabled.");
  delay(Delays::LONG); // Allow power to stabilize

  // // Initialize Reset Pin for LoRa Module
  pinMode(Pins::LORA_RST, OUTPUT);
  digitalWrite(Pins::LORA_RST, HIGH);
  delay(Delays::LONG);
  digitalWrite(Pins::LORA_RST, LOW); //
  delay(Delays::LONG);
  digitalWrite(Pins::LORA_RST, HIGH); // Ensure RESET is inactive
  // loraManager.sendCommand(LoRaCommands::SYS_RESET, Delays::LONG);
  // Serial.println("RN2903 RESET pin set to HIGH (inactive).");
  delay(Delays::LONG);

  // Reset the LoRa Module
  // resetLoRaModule();

  // Initialize OLED Display
  initializeOLED();

  // Initialize LoRa Communication
  loraManager.begin();

  // Initialize Modbus Communication
  modbusManager.begin();

  Serial.println("LoRa Module initialized with custom settings.");
  delay(Delays::LONG); // Wait for settings to take effect

  // Send initial command to get system version from LoRa
  // Serial.println("Requesting system version from RN2903...");
  // loraManager.sendCommand(LoRaCommands::SYS_GET_VER, Delays::LONG);
}

// ===========================
// Loop Function
// ===========================

/**
 * @brief Arduino loop function. Periodically reads data from Modbus, constructs JSON,
 *        sends it via LoRa, and displays relevant information on the OLED.
 */
/**
 * @brief Main loop function that handles periodic tasks.
 *
 * This function is called repeatedly and performs the following tasks:
 * - Receives data from LoRa and processes it.
 * - If USE_MOCK_DATA is false, reads various electrical parameters from Modbus registers:
 *   - Voltages (VA, VB, VC)
 *   - Currents (IA, IB, IC)
 *   - Total Apparent Power
 *   - Power Factor
 *   - Active Power (kW)
 *   - Total Active Power
 * - Constructs a JSON object with the read data and sends it via LoRa.
 * - Displays the data on an OLED screen.
 * - If USE_MOCK_DATA is true, uses predefined mock data instead of reading from Modbus.
 *
 * The function ensures that the tasks are performed at intervals defined by LOOP_INTERVAL.
 */
void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= LOOP_INTERVAL)
  {
    previousMillis = currentMillis;

    // Attempt to receive data from LoRa
    if (loraManager.receiveData(latestLoraData))
    {
      // Process LoRa data if needed
    }
    else
    {
      Serial.println("No data from LoRa.");
    }

    if (!USE_MOCK_DATA)
    {
      // Handle Modbus Communication
      // Define buffers for register data
      uint16_t buffer[2];
      bool success;

      // Read Voltage Registers VA, VB, VC
      uint16_t voltages[3];
      success = modbusManager.readRegisters(ModbusRegisters::VA, 3, voltages);
      if (success)
      {
        float va = voltages[0] * 0.01f;
        float vb = voltages[1] * 0.01f;
        float vc = voltages[2] * 0.01f;

        Serial.printf("Va: %.2f V\n", va);
        Serial.printf("Vb: %.2f V\n", vb);
        Serial.printf("Vc: %.2f V\n", vc);
      }

      // Read Current Registers IA, IB, IC (UINT32 each)
      float currents[3] = {0.0f, 0.0f, 0.0f};
      const uint16_t currentAddresses[3] = {ModbusRegisters::IA, ModbusRegisters::IB, ModbusRegisters::IC};
      for (int i = 0; i < 3; i++)
      {
        success = modbusManager.readRegisters(currentAddresses[i], 2, buffer);
        if (success)
        {
          uint32_t raw = ((uint32_t)buffer[1] << 16) | buffer[0]; // Little endian
          currents[i] = raw * 0.001f;                             // Scale factor
          Serial.printf("I%c: %.3f A\n", 'A' + i, currents[i]);
        }
      }

      // Read Total Apparent Power (VA)
      uint16_t totalVaBuffer[2];
      float totalVa = 0.0f;
      success = modbusManager.readRegisters(ModbusRegisters::TOTAL_APPARENT_POWER, 2, totalVaBuffer);
      if (success)
      {
        uint32_t raw = ((uint32_t)totalVaBuffer[1] << 16) | totalVaBuffer[0];
        totalVa = raw * 0.01f;
        Serial.printf("Total Apparent Power: %.2f VA\n", totalVa);
      }

      // Read Power Factor
      uint16_t powerFactorBuffer[1];
      float powerFactor = 0.0f;
      success = modbusManager.readRegisters(ModbusRegisters::POWER_FACTOR, 1, powerFactorBuffer);
      if (success)
      {
        int16_t raw = (int16_t)powerFactorBuffer[0];
        powerFactor = raw * 0.001f;
        Serial.printf("Power Factor: %.3f\n", powerFactor);
      }

      // Calculate Active Power (kW)
      float activePowerKW = 0.0f;
      if (success)
      { // Ensure Power Factor was read successfully
        activePowerKW = (totalVa * powerFactor) / 1000.0f;
        Serial.printf("Active Power: %.3f kW\n", activePowerKW);
      }

      // Read Total Active Power
      uint16_t totalActivePowerBuffer[2];
      float totalActivePower = 0.0f;
      success = modbusManager.readRegisters(ModbusRegisters::TOTAL_ACTIVE_POWER, 2, totalActivePowerBuffer);
      if (success)
      {
        uint32_t raw = ((uint32_t)totalActivePowerBuffer[1] << 16) | totalActivePowerBuffer[0];
        totalActivePower = raw * 0.01f;
        Serial.printf("Total Active Power: %.2f W\n", totalActivePower);
      }

      // ===========================
      // Prepare JSON Data to Send via LoRa
      // ===========================
      // Construct a JSON object as a string
      String jsonData = "{";
      jsonData += "\"" + String(JSONKeys::VA) + "\":" + String(voltages[0] * 0.01f, 2) + ",";
      jsonData += "\"" + String(JSONKeys::VB) + "\":" + String(voltages[1] * 0.01f, 2) + ",";
      jsonData += "\"" + String(JSONKeys::VC) + "\":" + String(voltages[2] * 0.01f, 2) + ",";
      jsonData += "\"" + String(JSONKeys::IA) + "\":" + String(currents[0], 3) + ",";
      jsonData += "\"" + String(JSONKeys::IB) + "\":" + String(currents[1], 3) + ",";
      jsonData += "\"" + String(JSONKeys::IC) + "\":" + String(currents[2], 3) + ",";
      jsonData += "\"" + String(JSONKeys::ACTIVE_POWER) + "\":" + String(activePowerKW, 3) + ",";
      jsonData += "\"" + String(JSONKeys::TOTAL_ACTIVE_POWER) + "\":" + String(totalActivePower * 0.001f, 2);
      jsonData += "}";

      Serial.println("JSON Data to send via LoRa: " + jsonData);

      // Convert JSON Data to Hex
      String hexData = stringToHex(jsonData);
      Serial.println("Hex Data: " + hexData);

      // Send Data via LoRa
      loraManager.sendData(hexData);

      // Display data on OLED
      displayData(jsonData, activePowerKW, totalActivePower);
    }
    else
    {
      // Use Mock Data
      // Example JSON mock data
      String mockJsonData = "{";
      mockJsonData += "\"" + String(JSONKeys::VA) + "\":230.00,";
      mockJsonData += "\"" + String(JSONKeys::VB) + "\":231.00,";
      mockJsonData += "\"" + String(JSONKeys::VC) + "\":229.50,";
      mockJsonData += "\"" + String(JSONKeys::IA) + "\":5.123,";
      mockJsonData += "\"" + String(JSONKeys::IB) + "\":5.456,";
      mockJsonData += "\"" + String(JSONKeys::IC) + "\":5.789,";
      mockJsonData += "\"" + String(JSONKeys::ACTIVE_POWER) + "\":12.345,";
      mockJsonData += "\"" + String(JSONKeys::TOTAL_ACTIVE_POWER) + "\":12.345";
      mockJsonData += "}";

      Serial.println("Using Mock Data: " + mockJsonData);

      // Convert Mock Data to Hex
      String mockHexData = stringToHex(mockJsonData);
      Serial.println("Mock Hex Data: " + mockHexData);

      // Send Mock Data via LoRa
      loraManager.sendData(mockHexData);
      Serial.println("Sending Mock Data via LoRa: " + String(LoRaCommands::RADIO_TX_PREFIX) + mockHexData);

      // Display Mock data on OLED
      float mockActivePowerKW = 12.345f;    /**< Example mock active power in kW */
      float mockTotalActivePower = 12.345f; /**< Example mock total active power in W */
      displayData(mockJsonData, mockActivePowerKW, mockTotalActivePower);
    }
  }

  // Optional: Handle incoming LoRa data if needed
}

// ===========================
// Function Implementations
// ===========================

/**
 * @brief Resets the RN2903 LoRa module by toggling the RESET pin.
 */
void resetLoRaModule()
{
  // Serial.println("Resetting RN2903 module...");
  // digitalWrite(Pins::LORA_RST, HIGH);                             // Deactivate RESET
  // delay(Delays::SHORT);                                           // Hold RESET high for a short duration
  // digitalWrite(Pins::LORA_RST, LOW);                              // Activate RESET
  // delay(Delays::LONG);                                            // Hold RESET low for 1 second
  // digitalWrite(Pins::LORA_RST, HIGH);                             // Deactivate RESET
  loraManager.sendCommand(LoRaCommands::SYS_RESET, Delays::LONG); // Reset LoRa module
  Serial.println("RN2903 module reset.");
  delay(Delays::LONG); // Wait for module to initialize after reset
}

/**
 * @brief Initializes the OLED display, displays a welcome message, and clears the screen.
 */
void initializeOLED()
{
  Serial.println("Initializing OLED display...");
  Wire.begin(Pins::OLED_SDA, Pins::OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // OLED_I2C_ADDRESS
    Serial.println("OLED initialization failed!");
    while (1)
      ; // Halt execution if OLED fails to initialize
  }
  Serial.println("OLED initialized successfully.");
  display.clearDisplay();
  display.setTextSize(OLEDConfig::TEXT_SIZE);
  display.setTextColor(OLEDConfig::TEXT_COLOR);
  display.setCursor(0, 0);
  display.println("ESP32, LoRa, Modbus");
  display.println("Setup Completed");
  display.display();
  delay(OLEDConfig::WELCOME_DELAY); // Display welcome message for configured duration
  display.clearDisplay();
}

/**
 * @brief Displays the total active power on the OLED screen.
 *
 * @param jsonData The JSON data string (unused in display but can be extended).
 * @param activePowerKW Active power in kilowatts.
 * @param totalActivePower Total active power in watts.
 */
void displayData(const String &jsonData, float activePowerKW, float totalActivePower)
{
  // Clear the display
  display.clearDisplay();

  // Display Total Active Power
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Total Active Power:");
  display.setTextSize(1.5);
  display.setCursor(0, 10);
  display.printf("%.2f kW\n", totalActivePower);

  display.display();
}
