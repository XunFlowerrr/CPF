#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>

// ===========================
// Pin Definitions
// ===========================

// LoRa Module Pins
#define LO_RA_RX_PIN 16  // RN2903 TX → ESP32 RX (UART2 RX)
#define LO_RA_TX_PIN 17  // RN2903 RX → ESP32 TX (UART2 TX)
#define LO_RA_RST_PIN 5  // RN2903 RESET
#define LO_RA_DIO0_PIN 4 // RN2903 DIO0 (Optional)
#define POWER_PIN 33     // Power supply to RN2903

// OLED Display Pins
#define OLED_SDA 21              // OLED SDA
#define OLED_SCL 22              // OLED SCL
#define SSD1306_I2C_ADDRESS 0x3C // OLED I2C Address

// Modbus Volt Meter Pins
#define MODBUS_RX_PIN 12 // Voltmeter TX → ESP32 RX (UART1 RX)
#define MODBUS_TX_PIN 18 // Voltmeter RX → ESP32 TX (UART1 TX)

// ===========================
// Serial Interfaces
// ===========================

// UART0: Serial Monitor (Built-in)
#define SERIAL_BAUD_RATE 115200

// UART1: Modbus Communication
HardwareSerial ModbusSerial(1);

// UART2: LoRa Communication
HardwareSerial LoRaSerial(2);

// ===========================
// OLED Display Initialization
// ===========================

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected to I2C (SDA, SCL)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===========================
// ModbusMaster Instance
// ===========================

ModbusMaster node;

// ===========================
// Register Addresses
// ===========================

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

// ===========================
// Global Variables
// ===========================

String latestLoraData = ""; // To store latest LoRa data

// ===========================
// Function Prototypes
// ===========================

void resetLoRaModule();
void initializeOLED();
void initializeLoRa();
void initializeModbus();
void displayData(String loraData, float activePowerKW, float totalActivePower);

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
  delay(100); // Allow power to stabilize

  // Initialize Reset Pin for LoRa Module
  pinMode(LO_RA_RST_PIN, OUTPUT);
  digitalWrite(LO_RA_RST_PIN, HIGH); // Ensure RESET is inactive
  Serial.println("RN2903 RESET pin set to HIGH (inactive).");
  delay(100);

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

  // Send initial command to get system version from LoRa
  Serial.println("Requesting system version from RN2903...");
  LoRaSerial.println("sys get ver"); // Lowercase command as per RN2903 specification
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

  // Handle Modbus Communication
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

  // Optionally, send this data via LoRa
  String dataToSend = "Va: " + String(va_raw * 0.01) + " V, " +
                      "Vb: " + String(vb_raw * 0.01) + " V, " +
                      "Vc: " + String(vc_raw * 0.01) + " V, " +
                      "Ia: " + String(ia) + " A, " +
                      "Ib: " + String(ib) + " A, " +
                      "Ic: " + String(ic) + " A, " +
                      "Active Power: " + String(active_power_kw) + " kW, " +
                      "Total Active Power: " + String(total_active_power) + " W";
  // LoRaSerial.println(dataToSend);

  // Display data on OLED
  displayData(dataToSend, active_power_kw, total_active_power);

  delay(2000); // Wait for 2 seconds before next loop
}

// ===========================
// Function Implementations
// ===========================

// Function to Reset the LoRa Module
void resetLoRaModule()
{
  Serial.println("Resetting RN2903 module...");
  digitalWrite(LO_RA_RST_PIN, LOW);  // Activate RESET
  delay(1000);                       // Hold RESET low for 1 second
  digitalWrite(LO_RA_RST_PIN, HIGH); // Deactivate RESET
  Serial.println("RN2903 module reset.");
  delay(2000); // Wait for module to initialize after reset
}

// Function to Initialize the OLED Display
void initializeOLED()
{
  Serial.println("Initializing OLED display...");
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS))
  {
    Serial.println("OLED initialization failed!");
    while (1)
      ; // Halt execution if OLED fails to initialize
  }
  Serial.println("OLED initialized successfully.");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESP32, LoRa, Modbus");
  display.println("Setup Completed");
  display.display();
  delay(2000); // Display welcome message for 2 seconds
}

// Function to Initialize LoRa Serial Communication
void initializeLoRa()
{
  Serial.println("Initializing LoRa Serial Communication...");
  LoRaSerial.begin(57600, SERIAL_8N1, LO_RA_RX_PIN, LO_RA_TX_PIN); // Initialize at 57600 bps
  delay(100);                                                      // Short delay to ensure serial starts
  Serial.println("LoRa Serial Communication initialized at 57600 bps.");

  // Optionally, verify if the module is responding
  Serial.println("Sending 'sys get ver' command to LoRa...");
  LoRaSerial.println("sys get ver"); // Lowercase command
  delay(1000);                       // Wait for response
}

// Function to Initialize Modbus Communication
void initializeModbus()
{
  Serial.println("Initializing Modbus Communication...");
  // Initialize ModbusSerial with the designated RX and TX pins
  ModbusSerial.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN); // Adjust baud rate as per volt meter
  delay(100);                                                         // Short delay to ensure serial starts
  Serial.println("Modbus Serial Communication initialized at 9600 bps.");
}

// Function to Display Data on OLED
void displayData(String loraData, float activePowerKW, float totalActivePower)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Display LoRa Data
  if (loraData.length() > 0)
  {
    display.println("LoRa Data:");
    display.println(loraData);
    display.println();
  }

  // Display Active Power
  display.println("Active Power:");
  display.print(activePowerKW);
  display.println(" kW");

  // Display Total Active Power
  display.println("Total Active Power:");
  display.print(totalActivePower);
  display.println(" W");

  display.display();
}
