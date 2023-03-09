/**
 * octopus_demo.cpp
 *
 * Functionalities:    Demo application to showcase the functionality of BME680 by configuring
 *                     the sensor in different modes of operation (ULP, LP and CONT) for different
 *                     data update rates at user level which in turn provides access to various use case.
 */

/** Global Library includes */
#include "bsec.h"
#include <EEPROM.h>

/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d
*/
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

/** Local Library includes  */

#include "OLED_featherwing_display.h"
/** Defines and consts */

// #define DEBUG_MODE

#define BUTTON_A    0       /**< Pin connected to OLED Featherwing on Octopus board */
#define BUTTON_C    2       /**< Pin connected to OLED Featherwing on Octopus board */

/* Save all 3 modes of operating parameters in EEPROM */

#define EEPROM_NUMBER_OF_REGIONS        3u
#define EEPROM_BYTES_PER_REGION         (BSEC_MAX_STATE_BLOB_SIZE + 1)    /* This value indicates max bytes per region including region identifier length*/

#define BSEC_EEPROM_ULP_IDENTIFIER      0u
#define BSEC_EEPROM_ULP_REGION_START    1u
#define BSEC_EEPROM_ULP_REGION_END      (EEPROM_BYTES_PER_REGION)

#define BSEC_EEPROM_LP_IDENTIFIER       BSEC_EEPROM_ULP_REGION_END
#define BSEC_EEPROM_LP_REGION_START     (BSEC_EEPROM_LP_IDENTIFIER + 1)
#define BSEC_EEPROM_LP_REGION_END       (BSEC_EEPROM_LP_REGION_START + EEPROM_BYTES_PER_REGION)

#define BSEC_EEPROM_CONT_IDENTIFIER       BSEC_EEPROM_LP_REGION_END
#define BSEC_EEPROM_CONT_REGION_START     (BSEC_EEPROM_CONT_IDENTIFIER + 1)
#define BSEC_EEPROM_CONT_REGION_END       (BSEC_EEPROM_CONT_REGION_START + EEPROM_BYTES_PER_REGION)

#define EEPROM_TOTAL_ALLOCATED_BYTES    (BSEC_EEPROM_CONT_REGION_END)

/** Structs and enums */
typedef enum
{
    ULP_MODE_SCREEN_OFF = 1,
    LP_MODE_SCREEN_ON = 2,
    CONT_MODE_BREATH = 3
} OPERATING_MODES;

/** Global variables */

OPERATING_MODES operating_mode = LP_MODE_SCREEN_ON;
static float previous_IAQ = 0;
static uint8_t previous_IAQ_acc = 0;

uint8_t bsec_state_ulp[BSEC_MAX_STATE_BLOB_SIZE] = { 0 };
uint8_t bsec_state_lp[BSEC_MAX_STATE_BLOB_SIZE] = { 0 };
uint8_t bsec_state_cont[BSEC_MAX_STATE_BLOB_SIZE] = { 0 };

bsec_virtual_sensor_t sensorList[5] = {
                                        BSEC_OUTPUT_RAW_TEMPERATURE,
                                        BSEC_OUTPUT_RAW_PRESSURE,
                                        BSEC_OUTPUT_RAW_HUMIDITY,
                                        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
                                        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
};

bsec_virtual_sensor_t sensorIAQ[2] = { 
                                        BSEC_OUTPUT_RAW_GAS,
                                        BSEC_OUTPUT_IAQ
};

String output;

// Create a Bsec object
Bsec iaqSensor;

/** Function declarations */
void ICACHE_RAM_ATTR button_press_A(void);
void ICACHE_RAM_ATTR button_press_C(void);
bool button_A_intr = false;
bool button_C_intr = false;
void eeprom_initialize(void);
void configure_sensor_mode(OPERATING_MODES operating_mode);
void report_status(void);
void err_leds(void);
void load_state(OPERATING_MODES operating_mode);
void update_state(OPERATING_MODES operating_mode);

/**
 *  @brief  Setup function for peripherals and other controls of MCU.
 *
 */
void setup()
{
    /*  Initialize Serial Terminal */
    Serial.begin(115200);
    /* Temporary place holder for firmware version */
    String bsecVersion;
    /*  Initialize I2C */
    Wire.begin();

    /*  Required delay for OLED display to get sufficient time after power on. */
    delay(1000);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.print("\nInitializing OLED Display");
    /* Initialize OLED display */
    initialize_display();

    display_print_string("Initializing...", 0, 0, 1);

    Serial.print("\nInitializing on board EEPROM");
    eeprom_initialize();

    Serial.print("\nInitializing buttons");
    /* Setup button interrupt to trigger scanning mode */
    pinMode(BUTTON_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_A), button_press_A, RISING);
    pinMode(BUTTON_C, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_C), button_press_C, RISING);
    Serial.print("\nInitializing Sensor connected on I2C Address: ");
    Serial.println(BME68X_I2C_ADDR_LOW);

    /* Sensor and Library initialization */
    configure_sensor_mode(operating_mode);

    /* Report loaded library on serial terminal */
    bsecVersion = "BSEC V" + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(bsecVersion);

    // Print the header
    output = "Timestamp [ms], raw temp[°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, comp temp[°C], comp humidity [%], operating mode, bme68x status, bsec status";
    Serial.println(output);

    display_bosch_logo(bsecVersion);
}

/**
 *  @brief  Arduino continuous looping function.
 */
void loop()
{
    String oled_output_line1, oled_output_line2_0, oled_output_line2;
    if (iaqSensor.run())
    {
        digitalWrite(LED_BUILTIN, LOW);
        output = String(millis());
        output += ", " + String(iaqSensor.rawTemperature);
        output += ", " + String(iaqSensor.pressure);
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.iaq);
        output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        output += ", " + String(operating_mode);
        output += ", " + String(iaqSensor.bme68xStatus);
        output += ", " + String(iaqSensor.bsecStatus);
        Serial.println(output);
        digitalWrite(LED_BUILTIN, HIGH);

        /* Keep updating sensor state values in RAM based on mode of operation */
        switch (operating_mode)
        {
            case ULP_MODE_SCREEN_OFF:

                iaqSensor.getState(bsec_state_ulp);

                /* Happens only during first time after power on */
                if (EEPROM.read(BSEC_EEPROM_ULP_IDENTIFIER) != BSEC_MAX_STATE_BLOB_SIZE)
                {
                    update_state(ULP_MODE_SCREEN_OFF);
                }

                oled_output_line2 = "Mode:ULP";
                break;

            case LP_MODE_SCREEN_ON:

                iaqSensor.getState(bsec_state_lp);

                /* Happens only during first time after power on */
                if (EEPROM.read(BSEC_EEPROM_LP_IDENTIFIER) != BSEC_MAX_STATE_BLOB_SIZE)
                {
                    update_state(LP_MODE_SCREEN_ON);
                }
                oled_output_line2 = "Mode:LP";
                break;

            case CONT_MODE_BREATH:

                iaqSensor.getState(bsec_state_cont);
                /* Happens only during first time after power on */
                if (EEPROM.read(BSEC_EEPROM_CONT_IDENTIFIER) != BSEC_MAX_STATE_BLOB_SIZE)
                {
                    update_state(CONT_MODE_BREATH);
                }
                oled_output_line2 = "Mode:CONT";
                break;

            default:
                break;

        }

        /* String handling segment for OLED display */
        if((iaqSensor.gasResistance == 0) && (iaqSensor.iaq == 0))
        {
            oled_output_line1 = "IAQ:" + String(previous_IAQ, 2);
            oled_output_line2_0 = "  " + String(previous_IAQ_acc);
        }
        else
        {
            oled_output_line1 = "IAQ:" + String(iaqSensor.iaq, 2);
            previous_IAQ = iaqSensor.iaq;
            oled_output_line2_0 = "  " + String(iaqSensor.iaqAccuracy);
            previous_IAQ_acc = iaqSensor.iaqAccuracy;
        }

        oled_output_line2 += "  degC:"
                             + String(iaqSensor.temperature, 1)
                             +
                             "\nrH:"
                             + String(iaqSensor.humidity, 1) +
                             "%  hPa:"
                             + String((iaqSensor.pressure / 100.0f), 1);

        display_print_multi_string(oled_output_line1, 0, 0, 2, oled_output_line2_0,
                                   oled_output_line2,
                                   0, 16, 1);

    }
    else{
        report_status();
    }

    /* Handle button press interrupt and trigger underlying functionality to switch modes */
    if (button_A_intr == true)
    {
        switch (operating_mode)
        {
            case ULP_MODE_SCREEN_OFF:

                /* Update current mode i.e save current values from RAM to EEPROM for LP Mode. */
                update_state(operating_mode);

                /* Toggle operating mode. */
                operating_mode = LP_MODE_SCREEN_ON;

                /* Load settings for new mode from EEPROM to RAM */
                load_state(operating_mode);

                /* Configure sensor operation for updated mode. */
                configure_sensor_mode(operating_mode);
                break;

            case LP_MODE_SCREEN_ON:

                /* Update current mode i.e save current values from RAM to EEPROM for LP Mode. */
                update_state(operating_mode);

                /* Toggle operating mode. */
                operating_mode = CONT_MODE_BREATH;

                /* Load settings for new mode from EEPROM to RAM */
                load_state(operating_mode);

                /* Configure sensor operation for updated mode. */
                configure_sensor_mode(operating_mode);
                break;

            case CONT_MODE_BREATH:

                /* Reached End of button functionality for this mode. No more further toggling allowed. */
                /* Update current mode i.e save current values from RAM to EEPROM for CONT Mode. */
                operating_mode = CONT_MODE_BREATH;
                update_state(operating_mode);
                break;

            default:
                break;

        }
        button_A_intr = false;
    }

    /* Handle button press interrupt and trigger underlying functionality to switch modes */
    if (button_C_intr)
    {
        switch (operating_mode)
        {
            case ULP_MODE_SCREEN_OFF:
                /* Reached End of button functionality for this mode. No more further toggling allowed. */
                /* Update current mode i.e save current values from RAM to EEPROM for ULP Mode. */
                /* Toggle operating mode. */
                operating_mode = ULP_MODE_SCREEN_OFF;

                update_state(operating_mode);
                break;

            case LP_MODE_SCREEN_ON:

                /* Update current mode i.e save current values from RAM to EEPROM for LP Mode. */
                update_state(operating_mode);

                /* Toggle operating mode. */
                operating_mode = ULP_MODE_SCREEN_OFF;

                /* Load settings for new mode from EEPROM to RAM */
                load_state(operating_mode);

                /* Configure sensor operation for updated mode. */
                configure_sensor_mode(operating_mode);
                break;

            case CONT_MODE_BREATH:

                /* Update current mode i.e save current values from RAM to EEPROM for CONT Mode. */
                update_state(operating_mode);

                /* Toggle operating mode. */
                operating_mode = LP_MODE_SCREEN_ON;

                /* Load settings for new mode from EEPROM to RAM */
                load_state(operating_mode);

                /* Configure sensor operation for updated mode. */
                configure_sensor_mode(operating_mode);
                break;

            default:
                break;

        }
        button_C_intr = false;
    }
}

/**
 * @brief   Button click A Handler
 *          Toggle operation:   ULP -> LP -> CONT(Breath Mode)
 *
 */
void ICACHE_RAM_ATTR button_press_A(void)
{
    button_A_intr = true;
}

/**
 * @brief   Button click C Handler
 *          Toggle operation:   CONT(Breath Mode) -> LP -> ULP
 *
 */
void ICACHE_RAM_ATTR button_press_C(void)
{
    button_C_intr = true;
}

/**
 * @brief   Configuration of bme680 and bsec library can be done using this function.
 * @param   operating_mode[in]   Mode of sensor operation. Can be chosen from the list below.
 *                           -ULP_MODE_SCREEN_OFF
 *                           -LP_MODE_SCREEN_ON
 *                           -CONT_MODE_BREATH
 */
void configure_sensor_mode(OPERATING_MODES operating_mode)
{
#ifdef DEBUG_MODE
    Serial.print("\nConfiguring sensor mode: ");
#endif
    /* Initialize BME68X sensor */
    iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);

    // Set config
    iaqSensor.setConfig(bsec_config_iaq);

    switch (operating_mode)
    {
        case ULP_MODE_SCREEN_OFF:
#ifdef DEBUG_MODE
            Serial.print("ULP Mode");
#endif
            iaqSensor.setState(bsec_state_ulp);
            iaqSensor.updateSubscription(sensorList, 5, BSEC_SAMPLE_RATE_LP);
            iaqSensor.updateSubscription(sensorIAQ, 2, BSEC_SAMPLE_RATE_ULP);
            break;

        case LP_MODE_SCREEN_ON:
#ifdef DEBUG_MODE
            Serial.print("LP Mode");
#endif
            iaqSensor.setState(bsec_state_lp);
            iaqSensor.updateSubscription(sensorList, 5, BSEC_SAMPLE_RATE_CONT);
            iaqSensor.updateSubscription(sensorIAQ, 2, BSEC_SAMPLE_RATE_LP);
            break;

        case CONT_MODE_BREATH:
#ifdef DEBUG_MODE
            Serial.print("CONT Mode");
#endif
            iaqSensor.setState(bsec_state_cont);
            iaqSensor.updateSubscription(sensorList, 5, BSEC_SAMPLE_RATE_CONT);
            iaqSensor.updateSubscription(sensorIAQ, 2, BSEC_SAMPLE_RATE_CONT);
            break;

        default:
            break;

    }
#ifdef DEBUG_MODE
    Serial.print("\nUpdate subscription: ");
    /* Report Sensor status on serial terminal */
    report_status();

#endif
}

/**
 * @brief   Initializes EEPROM memory space and loads previously saved values
 * @return  void
 */
void eeprom_initialize(void)
{
    bsec_state_ulp[BSEC_MAX_STATE_BLOB_SIZE-1] = 10;
    /*  Initialize EEPROM Memory segment of ESP8266 */
    EEPROM.begin(EEPROM_TOTAL_ALLOCATED_BYTES); // 1st address for the length

    if ((EEPROM.read(BSEC_EEPROM_ULP_IDENTIFIER) == BSEC_MAX_STATE_BLOB_SIZE) ||
        (EEPROM.read(BSEC_EEPROM_LP_IDENTIFIER) == BSEC_MAX_STATE_BLOB_SIZE)
        ||
        (EEPROM.read(BSEC_EEPROM_CONT_IDENTIFIER) == BSEC_MAX_STATE_BLOB_SIZE))
    {
        Serial.print("\nSaved data found, loading previous EEPROM values ");
        for (uint16_t array_index = 0; array_index < BSEC_MAX_STATE_BLOB_SIZE; array_index++)
        {
            bsec_state_ulp[array_index] = EEPROM.read(BSEC_EEPROM_ULP_REGION_START + array_index);
            bsec_state_lp[array_index] = EEPROM.read(BSEC_EEPROM_LP_REGION_START + array_index);
            bsec_state_cont[array_index] = EEPROM.read(BSEC_EEPROM_CONT_REGION_START + array_index);
        }
    }
    else
    {
        Serial.print("\nFirst time flash. Erasing complete EEPROM");
        for (uint16_t i = 0; i < EEPROM_TOTAL_ALLOCATED_BYTES; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();

    }
}

/**
 * @brief Save the values from RAM to appropriate memory location in EEPROM based on operating_mode
 * @param operating_mode    Defines the start EEPROM address for writing to EEPROM
 *                           -ULP_MODE_SCREEN_OFF
 *                           -LP_MODE_SCREEN_ON
 *                           -CONT_MODE_BREATH
 */
void update_state(OPERATING_MODES operating_mode)
{
#ifdef DEBUG_MODE
    Serial.print("\n Writing to EEPROM: ");
#endif
    switch (operating_mode)
    {
        case ULP_MODE_SCREEN_OFF:
#ifdef DEBUG_MODE
            Serial.print("ULP_MODE_SCREEN_OFF \n");
#endif
            for (uint16_t array_index = 0; array_index < BSEC_MAX_STATE_BLOB_SIZE; array_index++)
            {
                EEPROM.write(BSEC_EEPROM_ULP_REGION_START + array_index, bsec_state_ulp[array_index]);

#ifdef DEBUG_MODE
                Serial.print(EEPROM.read(BSEC_EEPROM_ULP_REGION_START + array_index));
                Serial.print(" ");

#endif
            }
            EEPROM.write(BSEC_EEPROM_ULP_IDENTIFIER, BSEC_MAX_STATE_BLOB_SIZE);
            break;

        case LP_MODE_SCREEN_ON:
#ifdef DEBUG_MODE
            Serial.print("LP_MODE_SCREEN_ON \n");
#endif
            for (uint16_t array_index = 0; array_index < BSEC_MAX_STATE_BLOB_SIZE; array_index++)
            {
                EEPROM.write(BSEC_EEPROM_LP_REGION_START + array_index, bsec_state_lp[array_index]);

#ifdef DEBUG_MODE
                Serial.print(EEPROM.read(BSEC_EEPROM_LP_REGION_START + array_index));
                Serial.print(" ");
#endif
            }
            EEPROM.write(BSEC_EEPROM_LP_IDENTIFIER, BSEC_MAX_STATE_BLOB_SIZE);

            break;

        case CONT_MODE_BREATH:
#ifdef DEBUG_MODE
            Serial.print("CONT_MODE_BREATH \n");
#endif
            for (uint16_t array_index = 0; array_index < EEPROM_BYTES_PER_REGION; array_index++)
            {
                EEPROM.write(BSEC_EEPROM_CONT_REGION_START + array_index, bsec_state_cont[array_index]);

#ifdef DEBUG_MODE
                Serial.print(EEPROM.read(BSEC_EEPROM_CONT_REGION_START + array_index));
                Serial.print(" ");
#endif
            }
            EEPROM.write(BSEC_EEPROM_CONT_IDENTIFIER, BSEC_MAX_STATE_BLOB_SIZE);
            break;

        default:
            break;
    }
    EEPROM.commit();
}

/**
 * @brief Load the EEPROM values into appropriate RAM containers based on
 *        operating_mode
 * @param operating_mode    Defines the start EEPROM address for reading from EEPROM
 *                           -ULP_MODE_SCREEN_OFF
 *                           -LP_MODE_SCREEN_ON
 *                           -CONT_MODE_BREATH
 */
void load_state(OPERATING_MODES operating_mode)
{
#ifdef DEBUG_MODE
    Serial.print("\n Loading from EEPROM: ");
#endif
    switch (operating_mode)
    {
        case ULP_MODE_SCREEN_OFF:
#ifdef DEBUG_MODE
            Serial.print("ULP_MODE_SCREEN_OFF \n");
#endif
            for (uint16_t array_index = 0; array_index < EEPROM_BYTES_PER_REGION; array_index++)
            {
                bsec_state_ulp[array_index] = EEPROM.read(BSEC_EEPROM_ULP_REGION_START + array_index);

#ifdef DEBUG_MODE
                Serial.print(bsec_state_ulp[array_index]);
                Serial.print(" ");
#endif
            }
            break;

        case LP_MODE_SCREEN_ON:
#ifdef DEBUG_MODE
            Serial.print("LP_MODE_SCREEN_ON \n");
#endif
            for (uint16_t array_index = 0; array_index < EEPROM_BYTES_PER_REGION; array_index++)
            {
                bsec_state_lp[array_index] = EEPROM.read(BSEC_EEPROM_LP_REGION_START + array_index);

#ifdef DEBUG_MODE
                Serial.print(bsec_state_lp[array_index]);
                Serial.print(" ");
#endif
            }
            break;

        case CONT_MODE_BREATH:
#ifdef DEBUG_MODE
            Serial.print("CONT_MODE_BREATH \n");
#endif
            for (uint16_t array_index = 0; array_index < EEPROM_BYTES_PER_REGION; array_index++)
            {
                bsec_state_cont[array_index] = EEPROM.read(BSEC_EEPROM_CONT_REGION_START + array_index);

#ifdef DEBUG_MODE
                Serial.print(bsec_state_cont[array_index]);
                Serial.print(" ");
#endif
            }
            break;

        default:
            break;
    }
}

/**
 *  @brief  Report the current status of bme680 and bsec library.
 */
void report_status(void)
{
    /* Report Sensor status on serial terminal */
    if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "\nBSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      for (;;)
        err_leds(); /* Halt in case of failure */
    } else {
      output = "\nBSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "\nBME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
      for (;;)
        err_leds(); /* Halt in case of failure */
    } else {
      output = "\nBME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}

void err_leds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
