#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>

// ===========================
// Configuration Section
// ===========================

// ----- Pin Definitions -----

// LoRa Module Pins
#define LORA_RX_PIN 16  // RN2903 TX → ESP32 RX (UART2 RX)
#define LORA_TX_PIN 17  // RN2903 RX → ESP32 TX (UART2 TX)
#define LORA_RST_PIN 5  // RN2903 RESET
#define LORA_DIO0_PIN 4 // RN2903 DIO0 (Optional)
#define POWER_PIN 33    // Power supply to RN2903

// OLED Display Pins
#define OLED_SDA 21           // OLED SDA
#define OLED_SCL 22           // OLED SCL
#define OLED_I2C_ADDRESS 0x3C // OLED I2C Address

// Modbus Volt Meter Pins
#define MODBUS_RX_PIN 12 // Voltmeter TX → ESP32 RX (UART1 RX)
#define MODBUS_TX_PIN 18 // Voltmeter RX → ESP32 TX (UART1 TX)

// ----- Serial Interfaces -----

// UART0: Serial Monitor (Built-in)
#define SERIAL_BAUD_RATE 115200

// UART1: Modbus Communication
#define MODBUS_SERIAL_PORT 1

// UART2: LoRa Communication
#define LORA_SERIAL_PORT 2

// ----- OLED Display Settings -----

#define SCREEN_WIDTH 128              // OLED display width, in pixels
#define SCREEN_HEIGHT 64              // OLED display height, in pixels
#define OLED_TEXT_SIZE 1              // Text size for OLED
#define OLED_TEXT_COLOR SSD1306_WHITE // Text color for OLED
#define OLED_WELCOME_DELAY 2000       // Delay to display welcome message (ms)

// ----- Modbus Register Addresses -----

// Voltage Registers
#define REG_VA 0 // 40001 - 0
#define REG_VB 1 // 40002 - 1
#define REG_VC 2 // 40003 - 2

// Current Registers
#define REG_IA 3 // 40004 - 3 (UINT32)
#define REG_IB 5 // 40006 - 5 (UINT32)
#define REG_IC 7 // 40008 - 7 (UINT32)

// Power Registers
#define REG_TOTAL_APPARENT_POWER 11 // 40012 - 11 (LUINT32)
#define REG_POWER_FACTOR 10         // 40011 - 10 (INT16)
#define REG_TOTAL_ACTIVE_POWER 23   // 40024 - 23 (0.01 W)

// ----- Configuration Toggles -----

#define USE_MOCK_DATA true // Set to true to use mock data instead of actual data

// ----- Delay Durations (ms) -----

#define DELAY_SHORT 100
#define DELAY_MEDIUM 500
#define DELAY_LONG 1000
#define DELAY_EXTRA_LONG 2000

// ----- LoRa Commands -----

#define LORA_CMD_MAC_PAUSE "mac pause"
#define LORA_CMD_RADIO_BW "radio set bw 125"
#define LORA_CMD_RADIO_CR "radio set cr 4/5"
#define LORA_CMD_RADIO_PWR "radio set pwr 20"
#define LORA_CMD_RADIO_FREQ "radio set freq 910000000"
#define LORA_CMD_RADIO_SF "radio set sf sf7"
#define LORA_CMD_SYS_GET_VER "sys get ver"
#define LORA_CMD_RADIO_TX_PREFIX "radio tx "

// ----- JSON Configuration -----

#define JSON_VA_KEY "Va"
#define JSON_VB_KEY "Vb"
#define JSON_VC_KEY "Vc"
#define JSON_IA_KEY "Ia"
#define JSON_IB_KEY "Ib"
#define JSON_IC_KEY "Ic"
#define JSON_ACTIVE_POWER_KEY "ActivePower"
#define JSON_TOTAL_ACTIVE_POWER_KEY "TotalActivePower"

// ===========================
// Global Objects and Variables
// ===========================

// OLED Display Instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ModbusMaster Instance
ModbusMaster node;

// HardwareSerial Instances
HardwareSerial ModbusSerial(MODBUS_SERIAL_PORT);
HardwareSerial LoRaSerial(LORA_SERIAL_PORT);

// Global Variables
String latestLoraData = ""; // To store latest LoRa data

// ===========================
// Function Prototypes
// ===========================

void resetLoRaModule();
void initializeOLED();
void initializeLoRa();
void initializeModbus();
void displayData(String jsonData, float activePowerKW, float totalActivePower);
String stringToHex(String input);
String hexToString(String hex);

// ===========================
// Setup Function
// ===========================

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
  {
    ; // Wait for Serial Monitor to open
  }
  Serial.println("==================================");
  Serial.println("ESP32, RN2903 LoRa, and Modbus Setup");
  Serial.println("==================================");

  // Initialize Power Pin for LoRa Module
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH); // Turn on RN2903
  Serial.println("Power to RN2903 enabled.");
  delay(DELAY_SHORT); // Allow power to stabilize

  // Initialize Reset Pin for LoRa Module
  pinMode(LORA_RST_PIN, OUTPUT);
  digitalWrite(LORA_RST_PIN, HIGH); // Ensure RESET is inactive
  Serial.println("RN2903 RESET pin set to HIGH (inactive).");
  delay(DELAY_SHORT);

  // Reset the LoRa Module
  resetLoRaModule();

  // Initialize OLED Display
  initializeOLED();

  // Initialize LoRa Serial Communication
  initializeLoRa();

  // Initialize Modbus Communication
  initializeModbus();

  // Initialize ModbusMaster
  node.begin(1, ModbusSerial); // Slave ID 1

  Serial.println("LoRa Module initialized with custom settings.");
  delay(DELAY_LONG); // Wait for settings to take effect

  // Send initial command to get system version from LoRa
  Serial.println("Requesting system version from RN2903...");
  LoRaSerial.println(LORA_CMD_SYS_GET_VER); // Lowercase command as per RN2903 specification
  delay(DELAY_LONG);                        // Wait for response
}

// ===========================
// Loop Function
// ===========================

void loop()
{
  // Handle LoRa Communication
  if (LoRaSerial.available())
  {
    String loraResponse = LoRaSerial.readStringUntil('\n');
    loraResponse.trim(); // Remove any trailing whitespace or newline characters
    Serial.print("Received from LoRa: ");
    Serial.println(loraResponse);
    latestLoraData = loraResponse; // Update latest LoRa data
  }
  else
  {
    Serial.println("No data from LoRa.");
  }

  // Handle Modbus Communication only if not using mock data
  if (!USE_MOCK_DATA)
  {
    uint8_t result;

    // Reading Voltages Va, Vb, Vc (Registers 40001, 40002, 40003)
    uint16_t va_raw, vb_raw, vc_raw;
    result = node.readHoldingRegisters(REG_VA, 3); // Read 3 registers starting at REG_VA
    if (result == node.ku8MBSuccess)
    {
      va_raw = node.getResponseBuffer(0);
      vb_raw = node.getResponseBuffer(1);
      vc_raw = node.getResponseBuffer(2);

      float va = va_raw * 0.01;
      float vb = vb_raw * 0.01;
      float vc = vc_raw * 0.01;

      Serial.print("Va: ");
      Serial.print(va);
      Serial.println(" V");
      Serial.print("Vb: ");
      Serial.print(vb);
      Serial.println(" V");
      Serial.print("Vc: ");
      Serial.print(vc);
      Serial.println(" V");
    }
    else
    {
      Serial.print("Modbus Read Error (Voltages): ");
      Serial.println(result, HEX);
    }

    // Reading Currents Ia, Ib, Ic (Registers 40004, 40006, 40008)
    uint32_t ia_raw, ib_raw, ic_raw;

    // Read Ia (40004)
    result = node.readHoldingRegisters(REG_IA, 2); // UINT32 occupies 2 registers
    float ia = 0.0;
    if (result == node.ku8MBSuccess)
    {
      ia_raw = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0); // Swap for little endian
      ia = ia_raw * 0.001;                                                              // Scale factor
      Serial.print("Ia: ");
      Serial.print(ia);
      Serial.println(" A");
    }
    else
    {
      Serial.print("Modbus Read Error (Ia): ");
      Serial.println(result, HEX);
    }

    // Read Ib (40006)
    result = node.readHoldingRegisters(REG_IB, 2); // UINT32 occupies 2 registers
    float ib = 0.0;
    if (result == node.ku8MBSuccess)
    {
      ib_raw = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0); // Swap for little endian
      ib = ib_raw * 0.001;                                                              // Scale factor
      Serial.print("Ib: ");
      Serial.print(ib);
      Serial.println(" A");
    }
    else
    {
      Serial.print("Modbus Read Error (Ib): ");
      Serial.println(result, HEX);
    }

    // Read Ic (40008)
    result = node.readHoldingRegisters(REG_IC, 2); // UINT32 occupies 2 registers
    float ic = 0.0;
    if (result == node.ku8MBSuccess)
    {
      ic_raw = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0); // Swap for little endian
      ic = ic_raw * 0.001;                                                              // Scale factor
      Serial.print("Ic: ");
      Serial.print(ic);
      Serial.println(" A");
    }
    else
    {
      Serial.print("Modbus Read Error (Ic): ");
      Serial.println(result, HEX);
    }

    // Reading Total Apparent Power (VA) (Register 40012)
    uint32_t total_va_raw = 0;
    float total_va = 0.0;
    result = node.readHoldingRegisters(REG_TOTAL_APPARENT_POWER, 2); // LUINT32 occupies 2 registers
    if (result == node.ku8MBSuccess)
    {
      total_va_raw = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0); // Swap for little endian
      total_va = total_va_raw * 0.01;                                                         // Scale factor
      Serial.print("Total Apparent Power: ");
      Serial.print(total_va);
      Serial.println(" VA");
    }
    else
    {
      Serial.print("Modbus Read Error (Total VA): ");
      Serial.println(result, HEX);
    }

    // Reading Power Factor (40011)
    int16_t power_factor_raw = 0;
    float power_factor = 0.0;
    result = node.readHoldingRegisters(REG_POWER_FACTOR, 1); // 40011 - 10 (INT16)
    if (result == node.ku8MBSuccess)
    {
      power_factor_raw = node.getResponseBuffer(0);
      power_factor = power_factor_raw * 0.001; // Scale factor
      Serial.print("Power Factor: ");
      Serial.println(power_factor);
    }
    else
    {
      Serial.print("Modbus Read Error (Power Factor): ");
      Serial.println(result, HEX);
    }

    // Calculate Active Power (kW) = Total Apparent Power (VA) * Power Factor / 1000
    float active_power_kw = 0.0;
    if (result == node.ku8MBSuccess) // Ensure Power Factor was read successfully
    {
      active_power_kw = (total_va * power_factor) / 1000.0;
      Serial.print("Active Power: ");
      Serial.print(active_power_kw);
      Serial.println(" kW");
    }

    // Reading Total Active Power (40024-40025)
    uint32_t total_active_power_raw = 0;
    float total_active_power = 0.0;
    result = node.readHoldingRegisters(REG_TOTAL_ACTIVE_POWER, 2); // Read 2 registers
    if (result == node.ku8MBSuccess)
    {
      total_active_power_raw = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0); // Swap for little endian
      total_active_power = total_active_power_raw * 0.01;                                               // Scale factor
      Serial.print("Total Active Power: ");
      Serial.print(total_active_power);
      Serial.println(" W");
    }
    else
    {
      Serial.print("Modbus Read Error (Total Active Power): ");
      Serial.println(result, HEX);
    }

    // ===========================
    // Prepare JSON Data to Send via LoRa
    // ===========================
    // Construct a JSON object as a string
    String jsonData = "{";
    jsonData += "\"" + String(JSON_VA_KEY) + "\":" + String(va_raw * 0.01, 2) + ",";
    jsonData += "\"" + String(JSON_VB_KEY) + "\":" + String(vb_raw * 0.01, 2) + ",";
    jsonData += "\"" + String(JSON_VC_KEY) + "\":" + String(vc_raw * 0.01, 2) + ",";
    jsonData += "\"" + String(JSON_IA_KEY) + "\":" + String(ia, 3) + ",";
    jsonData += "\"" + String(JSON_IB_KEY) + "\":" + String(ib, 3) + ",";
    jsonData += "\"" + String(JSON_IC_KEY) + "\":" + String(ic, 3) + ",";
    jsonData += "\"" + String(JSON_ACTIVE_POWER_KEY) + "\":" + String(active_power_kw, 3) + ",";
    jsonData += "\"" + String(JSON_TOTAL_ACTIVE_POWER_KEY) + "\":" + String(total_active_power, 2);
    jsonData += "}";

    Serial.println("JSON Data to send via LoRa: " + jsonData);

    // Convert JSON Data to Hex
    String hexData = stringToHex(jsonData);
    Serial.println("Hex Data: " + hexData);

    // Send Data via LoRa
    String loraCommand = String(LORA_CMD_RADIO_TX_PREFIX) + hexData;
    Serial.println("Sending via LoRa: " + loraCommand);
    LoRaSerial.println(loraCommand); // Send hex message
    delay(DELAY_SHORT);              // Short delay to ensure command is processed

    // Display data on OLED
    displayData(jsonData, active_power_kw, total_active_power);
  }
  else
  {
    // Use Mock Data
    // Example JSON mock data
    String mockJsonData = "{";
    mockJsonData += "\"" + String(JSON_VA_KEY) + "\":230.00,";
    mockJsonData += "\"" + String(JSON_VB_KEY) + "\":231.00,";
    mockJsonData += "\"" + String(JSON_VC_KEY) + "\":229.50,";
    mockJsonData += "\"" + String(JSON_IA_KEY) + "\":5.123,";
    mockJsonData += "\"" + String(JSON_IB_KEY) + "\":5.456,";
    mockJsonData += "\"" + String(JSON_IC_KEY) + "\":5.789,";
    mockJsonData += "\"" + String(JSON_ACTIVE_POWER_KEY) + "\":12.345,";
    mockJsonData += "\"" + String(JSON_TOTAL_ACTIVE_POWER_KEY) + "\":12345.67";
    mockJsonData += "}";

    Serial.println("Using Mock Data: " + mockJsonData);

    // Convert Mock Data to Hex
    String mockHexData = stringToHex(mockJsonData);
    Serial.println("Mock Hex Data: " + mockHexData);

    // Send Mock Data via LoRa
    String mockLoraCommand = String(LORA_CMD_RADIO_TX_PREFIX) + mockHexData;
    Serial.println("Sending Mock Data via LoRa: " + mockLoraCommand);
    LoRaSerial.println(mockLoraCommand); // Send hex mock message
    delay(DELAY_SHORT);                  // Short delay to ensure command is processed

    // Display Mock data on OLED
    float mockActivePowerKW = 12.345;      // Example mock value
    float mockTotalActivePower = 12345.67; // Example mock value
    displayData(mockJsonData, mockActivePowerKW, mockTotalActivePower);

    delay(DELAY_EXTRA_LONG); // Wait for 2 seconds before next loop
    return;                  // Skip the rest of the loop when using mock data
  }

  delay(DELAY_EXTRA_LONG); // Wait for 2 seconds before next loop
}

// ===========================
// Function Implementations
// ===========================

// Function to Reset the LoRa Module
void resetLoRaModule()
{
  Serial.println("Resetting RN2903 module...");
  digitalWrite(LORA_RST_PIN, HIGH); // Deactivate RESET
  delay(DELAY_SHORT);               // Hold RESET high for a short duration
  digitalWrite(LORA_RST_PIN, LOW);  // Activate RESET
  delay(DELAY_LONG);                // Hold RESET low for 1 second
  digitalWrite(LORA_RST_PIN, HIGH); // Deactivate RESET
  Serial.println("RN2903 module reset.");
  delay(DELAY_LONG); // Wait for module to initialize after reset
}

// Function to Initialize the OLED Display
void initializeOLED()
{
  Serial.println("Initializing OLED display...");
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS))
  {
    Serial.println("OLED initialization failed!");
    while (1)
      ; // Halt execution if OLED fails to initialize
  }
  Serial.println("OLED initialized successfully.");
  display.clearDisplay();
  display.setTextSize(OLED_TEXT_SIZE);
  display.setTextColor(OLED_TEXT_COLOR);
  display.setCursor(0, 0);
  display.println("ESP32, LoRa, Modbus");
  display.println("Setup Completed");
  display.display();
  delay(OLED_WELCOME_DELAY); // Display welcome message for configured duration
  display.clearDisplay();
}

// Function to Initialize LoRa Serial Communication
void initializeLoRa()
{
  Serial.println("Initializing LoRa Serial Communication...");
  LoRaSerial.begin(57600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN); // Initialize at 57600 bps
  delay(DELAY_SHORT);                                            // Short delay to ensure serial starts
  Serial.println("LoRa Serial Communication initialized at 57600 bps.");

  // Optionally, verify if the module is responding
  Serial.println("Sending 'sys get ver' command to LoRa...");
  LoRaSerial.println(LORA_CMD_SYS_GET_VER); // Lowercase command
  delay(DELAY_LONG);                        // Wait for response

  // Send initialization commands to LoRa
  Serial.println("Initializing LoRa Module with custom settings...");

  // Send each command and wait for acknowledgment
  LoRaSerial.println(LORA_CMD_MAC_PAUSE);
  delay(DELAY_MEDIUM);

  LoRaSerial.println(LORA_CMD_RADIO_BW);
  delay(DELAY_MEDIUM);

  LoRaSerial.println(LORA_CMD_RADIO_CR);
  delay(DELAY_MEDIUM);

  LoRaSerial.println(LORA_CMD_RADIO_PWR);
  delay(DELAY_MEDIUM);

  LoRaSerial.println(LORA_CMD_RADIO_FREQ);
  delay(DELAY_MEDIUM);

  LoRaSerial.println(LORA_CMD_RADIO_SF);
  delay(DELAY_MEDIUM);
}

// Function to Initialize Modbus Communication
void initializeModbus()
{
  Serial.println("Initializing Modbus Communication...");
  // Initialize ModbusSerial with the designated RX and TX pins
  ModbusSerial.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN); // Adjust baud rate as per volt meter
  delay(DELAY_SHORT);                                                 // Short delay to ensure serial starts
  Serial.println("Modbus Serial Communication initialized at 9600 bps.");
}

// Function to Display Data on OLED
void displayData(String jsonData, float activePowerKW, float totalActivePower)
{
  static int lineCount = 0; // Track the number of lines displayed

  // Parse JSON Data to Extract Relevant Fields (Optional)
  // For simplicity, we're displaying the entire JSON string.
  // For more advanced display, consider parsing the JSON.

  display.setTextSize(OLED_TEXT_SIZE);
  display.setTextColor(OLED_TEXT_COLOR);
  display.setCursor(0, lineCount * 10); // Move cursor to the next line
  display.print(totalActivePower);
  display.println(" W");
  lineCount++;

  // Optionally, display more fields from JSON if desired

  // Clear display when full
  if (lineCount >= 6)
  { // Adjust based on display height
    display.clearDisplay();
    lineCount = 0;
  }

  display.display();
}

// Function to convert a string to its hexadecimal representation
String stringToHex(String input)
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

// Function to convert a hexadecimal string back to a regular string
String hexToString(String hex)
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
