#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* I2C Address */
#define OLED_I2C_ADDRESS 0x3C // Change to 0x3D if necessary

/* OLED Dimensions */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

/* I2C Pins for ESP32 */
#define SDA_PIN 21
#define SCL_PIN 22

/* Create display instance */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Minimal SSD1306 OLED Test");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  delay(1000); // Allow some time for I2C to stabilize

  // Initialize the OLED display
  Serial.println("Initializing OLED...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS))
  {
    Serial.println("SSD1306 OLED initialization failed!");
    while (1)
      ; // Halt execution
  }
  Serial.println("OLED initialized successfully.");

  // Clear the buffer
  display.clearDisplay();
  Serial.println("Display buffer cleared.");

  // Set text properties and display a message
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hello, ESP32!");
  display.display();
  Serial.println("Displayed 'Hello, ESP32!'");
}

void loop()
{
  // Nothing to do here
}
