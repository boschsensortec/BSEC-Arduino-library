#include <EEPROM.h>
#include "bsec.h"
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

#define STATE_SAVE_PERIOD	UINT32_C(300000) // 360 minutes - 4 times a day
#define N_SENSORS			2

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

// Create objects of the class Bsec
Bsec iaqSensor1, iaqSensor2;
uint8_t bsecState1[BSEC_MAX_STATE_BLOB_SIZE], bsecState2[BSEC_MAX_STATE_BLOB_SIZE];
uint16_t stateUpdateCounter = 0;

String output;

// Entry point for the example
void setup(void)
{
  EEPROM.begin((N_SENSORS * BSEC_MAX_STATE_BLOB_SIZE) + 1); // 1st address for the length of one state
  Serial.begin(115200);
  Wire.begin();

  iaqSensor1.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  iaqSensor2.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor1.version.major) + "." + String(iaqSensor1.version.minor) + "." + String(iaqSensor1.version.major_bugfix) + "." + String(iaqSensor1.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  iaqSensor1.setConfig(bsec_config_iaq);
  iaqSensor2.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  loadState();

  bsec_virtual_sensor_t sensorList[7] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor1.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  iaqSensor2.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Sensor, Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%]";
  Serial.println(output);
}

// Function that is looped forever
void loop(void)
{
  unsigned long time_trigger = millis();
  if (iaqSensor1.run()) { // If new data is available
    output = "1, " + String(time_trigger);
    output += ", " + String(iaqSensor1.rawTemperature);
    output += ", " + String(iaqSensor1.pressure);
    output += ", " + String(iaqSensor1.rawHumidity);
    output += ", " + String(iaqSensor1.gasResistance);
    output += ", " + String(iaqSensor1.iaq);
    output += ", " + String(iaqSensor1.iaqAccuracy);
    output += ", " + String(iaqSensor1.temperature);
    output += ", " + String(iaqSensor1.humidity);
    Serial.println(output);
    updateState();
  } else {
    checkIaqSensorStatus();
  }

  time_trigger = millis();
  if (iaqSensor2.run()) { // If new data is available
    output = "2, " + String(time_trigger);
    output += ", " + String(iaqSensor2.rawTemperature);
    output += ", " + String(iaqSensor2.pressure);
    output += ", " + String(iaqSensor2.rawHumidity);
    output += ", " + String(iaqSensor2.gasResistance);
    output += ", " + String(iaqSensor2.iaq);
    output += ", " + String(iaqSensor2.iaqAccuracy);
    output += ", " + String(iaqSensor2.temperature);
    output += ", " + String(iaqSensor2.humidity);
    Serial.println(output);
    updateState();
  } else {
    checkIaqSensorStatus();
  }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor1.status != BSEC_OK) {
    if (iaqSensor1.status < BSEC_OK) {
      output = "BSEC1 error code : " + String(iaqSensor1.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor1.status);
      Serial.println(output);
    }
  }

  if (iaqSensor1.bme680Status != BME680_OK) {
    if (iaqSensor1.bme680Status < BME680_OK) {
      output = "BME6801 error code : " + String(iaqSensor1.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor1.bme680Status);
      Serial.println(output);
    }
  }
  iaqSensor1.status = BSEC_OK;

  if (iaqSensor2.status != BSEC_OK) {
    if (iaqSensor2.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor2.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor2.status);
      Serial.println(output);
    }
  }

  if (iaqSensor2.bme680Status != BME680_OK) {
    if (iaqSensor2.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor2.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor2.bme680Status);
      Serial.println(output);
    }
  }
  iaqSensor2.status = BSEC_OK;
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState1[i] = EEPROM.read(i + 1);
      bsecState2[i] = EEPROM.read(i + 1 + BSEC_MAX_STATE_BLOB_SIZE);
      Serial.println(String(bsecState1[i], HEX) + ", " + String(bsecState2[i], HEX));
    }

    iaqSensor1.setState(bsecState1);
    iaqSensor2.setState(bsecState2);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint16_t i = 0; i < (N_SENSORS * BSEC_MAX_STATE_BLOB_SIZE) + 1; i++) {
      EEPROM.write(i, 0);
    }

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if ((iaqSensor1.iaqAccuracy >= 3) || (iaqSensor2.iaqAccuracy >= 3)) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor1.getState(bsecState1);
    iaqSensor2.getState(bsecState2);
    checkIaqSensorStatus();

    Serial.println("Writing state(s) to EEPROM");

    for (uint16_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState1[i]);
      EEPROM.write(i + 1 + BSEC_MAX_STATE_BLOB_SIZE, bsecState2[i]);
      Serial.println(String(bsecState1[i], HEX) + ", " + String(bsecState2[i], HEX));
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}