/** Global Library includes */

#ifndef __OLED_FEATHERWING_DISPLAY_H_
#define __OLED_FEATHERWING_DISPLAY_H_

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

/** Local Library includes  */

#include "logos.h"

/** Defines and consts */
#define NUMBER_OF_IMAGES    5u
#define DISPLAY_I2C_ADDR    0x3c

/** Structs and enums */

struct ImageList
{
    uint8_t image[128];
};

extern ImageList image_list[NUMBER_OF_IMAGES];

/** Function declarations */

/**
 * @brief   Function used to initialize local objects for images with
 *          global consts from logos.h file
 */
void initialize_display();

/**
 * @brief   Function used to clear display and print the input string
 * @param   text    [input]    Input string that needs to be printed on display.
 */
void display_print_string(String text, uint8_t x_axis, uint8_t y_axis, uint8_t text_size);

/**
 * @brief   Function used to clear display and print the input string
 * @param   text    [input]    Input string that needs to be printed on display.
 */
void display_print_multi_string(String line1, uint8_t x_axis1, uint8_t y_axis1, uint8_t text_size1,
                                String line2_0,
                                String line2, uint8_t x_axis2, uint8_t y_axis2, uint8_t text_size2);

/**
 * @brief   Function used to display BOSCH logo on OLED display.
 * @param   fwVersion [input]   Firmware version that needs to be printed on OLED Display
 */
void display_bosch_logo(String fwVersion);

/**
 * @brief   Function used to clear display and print input label and string based on bmp Index.
 * @param   label   [input]     Title which needs to be printed on display
 * @param   value   [input]     Value which needs to be printed on display
 * @param   bmp     [input]     Index number corresponding to bmp files
 * @param   label_size [input]  Size of label string to be printed on OLED display
 * @param   value_size [input]  Size of value string to be printed on OLED display
 */
void display_print_img_string(String label, String value, uint8_t bmp, uint8_t label_size, uint8_t value_size);

/** Global variables */

extern Adafruit_SSD1306 display;

#endif
