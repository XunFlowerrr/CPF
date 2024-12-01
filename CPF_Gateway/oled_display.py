from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from PIL import Image, ImageDraw, ImageFont

class OLEDDisplay:
    def __init__(self, port=1, address=0x3C, width=128, height=64):
        self.serial = i2c(port=port, address=address)
        self.device = sh1106(self.serial, persist=True)
        self.width = width
        self.height = height
        self.font = ImageFont.load_default()  # Default font
        self.message = []
        self.max_lines = 6
    def display(self, text):
        """
        Display the given text on the OLED screen.


        :param text: Text string to display.
        """
        # Create a blank image
        self.message.append(text)
        if len(self.message) > self.max_lines:
            self.message.clear()
            self.message.append(text)
        image = Image.new("1", (self.width, self.height))
        draw = ImageDraw.Draw(image)

        # Calculate text position to center it
        text_bbox = draw.textbbox((0, 0), text, font=self.font)
        text_width, text_height = text_bbox[2] - text_bbox[0], text_bbox[3] - text_bbox[1]

        for i in range(len(self.message)):
            text = self.message[i]
            x = 0
            y = 0 + (i * 10)

            # Draw the text
            draw.text((x, y), text, font=self.font, fill=255)

        # Display the image on the OLED
        self.device.display(image)

    def clear(self):
        """
        Clear the OLED display.
        """
        self.device.clear()

