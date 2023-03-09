/** Global Library includes */

/** Local Library includes  */

#include "OLED_featherwing_display.h"

/** Global variables */
ImageList image_list[NUMBER_OF_IMAGES];

// Create an SSD1306 display object
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

/** Function definitions */

/********************************************************************************/
void initialize_display()
{
    // Decode logos from uint32_t x 32 (32x32) to uint8_t x 128
    decodeLogo(image_list[0].image, logo_bosch);
    decodeLogo(image_list[1].image, logo_temp);
    decodeLogo(image_list[2].image, logo_pres);
    decodeLogo(image_list[3].image, logo_hum);
    decodeLogo(image_list[4].image, logo_gas);
    // Initialize the OLED display
    display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDR); // initialize with the I2C addr 0x3C (for the 128x32)
    display.clearDisplay();
    display.display();
}

/********************************************************************************/
void display_print_string(String text, uint8_t x_axis, uint8_t y_axis, uint8_t text_size)
{
    display.clearDisplay();
    display.setTextSize(text_size);
    display.setTextColor(WHITE);
    display.setCursor(x_axis, y_axis);
    display.println(text);
    display.display();
}
/********************************************************************************/

void display_print_multi_string(String line1, uint8_t x_axis1, uint8_t y_axis1, uint8_t text_size1, String line2_0,
                                String line2,
                                uint8_t x_axis2, uint8_t y_axis2, uint8_t text_size2)
{
    display.clearDisplay();
    display.setTextSize(text_size1);
    display.setTextColor(WHITE);
    display.setCursor(x_axis1, y_axis1);
    display.println(line1);

    display.setTextSize(text_size2);
    display.setTextColor(WHITE);
    display.setCursor(x_axis2, y_axis2);
    display.println(line2);

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(110, 16);
    display.println("  A");
    display.setCursor(110, 24);
    display.println(line2_0);

    display.display();
}

/********************************************************************************/
void display_bosch_logo(String fwVersion)
{
    display.clearDisplay();
    display.drawBitmap(0, 0, image_list[0].image, 32, 32, WHITE);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(40, 10);
    display.println("BOSCH");
    display.setTextSize(1);
    display.setCursor(40, 00);
    display.println(fwVersion);
    display.display();
}

void display_print_img_string(String label, String value, uint8_t bmp, uint8_t label_size, uint8_t value_size)
{
    display.clearDisplay();
    display.drawBitmap(0, 0, image_list[bmp].image, 32, 32, WHITE);
    display.setTextSize(label_size);
    display.setTextColor(WHITE);
    display.setCursor(40, 0);
    display.println(label);
    display.setTextSize(value_size);
    display.setCursor(40, 20);
    display.println(value);
    display.display();
}

