/* Thanks to Dmitry for the sketch */
#include <Arduino.h>
#include <bsec.h>
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
#include <sys/time.h>

#define LOG(fmt, ...) (Serial.printf("%09llu: " fmt "\n", GetTimestamp(), ##__VA_ARGS__))

Bsec sensor;

RTC_DATA_ATTR uint8_t sensor_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
RTC_DATA_ATTR int64_t sensor_state_time = 0;

bsec_virtual_sensor_t sensor_list[] = {
  BSEC_OUTPUT_RAW_TEMPERATURE,
  BSEC_OUTPUT_RAW_PRESSURE,
  BSEC_OUTPUT_RAW_HUMIDITY,
  BSEC_OUTPUT_RAW_GAS,
  BSEC_OUTPUT_IAQ,
  BSEC_OUTPUT_STATIC_IAQ,
  BSEC_OUTPUT_CO2_EQUIVALENT,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
};

int64_t GetTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

bool CheckSensor() {
  if (sensor.status < BSEC_OK) {
    LOG("BSEC error, status %d!", sensor.status);
    return false;;
  } else if (sensor.status > BSEC_OK) {
    LOG("BSEC warning, status %d!", sensor.status);
  }

  if (sensor.bme680Status < BME680_OK) {
    LOG("Sensor error, bme680_status %d!", sensor.bme680Status);
    return false;
  } else if (sensor.bme680Status > BME680_OK) {
    LOG("Sensor warning, status %d!", sensor.bme680Status);
  }

  return true;
}

void DumpState(const char* name, const uint8_t* state) {
  LOG("%s:", name);
  for (int i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
    Serial.printf("%02x ", state[i]);
    if (i % 16 == 15) {
      Serial.print("\n");
    }
  }
  Serial.print("\n");
}

void setup() {
  Serial.begin(115200);
  SPI.begin();
  sensor.begin(2, SPI);
  if (!CheckSensor()) {
    LOG("Failed to init BME680, check wiring!");
    return;
  }

  LOG("BSEC version %d.%d.%d.%d", sensor.version.major, sensor.version.minor, sensor.version.major_bugfix, sensor.version.minor_bugfix);

  sensor.setConfig(bsec_config_iaq);
  if (!CheckSensor()) {
    LOG("Failed to set config!");
    return;
  }

  if (sensor_state_time) {
    DumpState("setState", sensor_state);
    sensor.setState(sensor_state);
    if (!CheckSensor()) {
      LOG("Failed to set state!");
      return;
    } else {
      LOG("Successfully set state from %lld", sensor_state_time);
    }
  } else {
    LOG("Saved state missing");
  }

  sensor.updateSubscription(sensor_list, sizeof(sensor_list) / sizeof(sensor_list[0]), BSEC_SAMPLE_RATE_LP);
  if (!CheckSensor()) {
    LOG("Failed to update subscription!");
    return;
  }

  LOG("Sensor init done");
}

void loop() {
  if (sensor.run(GetTimestamp())) {
    LOG("Temperature raw %.2f compensated %.2f", sensor.rawTemperature, sensor.temperature);
    LOG("Humidity raw %.2f compensated %.2f", sensor.rawHumidity, sensor.humidity);
    LOG("Pressure %.2f kPa", sensor.pressure / 1000);
    LOG("IAQ %.0f accuracy %d", sensor.iaq, sensor.iaqAccuracy);
    LOG("Static IAQ %.0f accuracy %d", sensor.staticIaq, sensor.staticIaqAccuracy);
    LOG("Gas resistance %.2f kOhm", sensor.gasResistance / 1000);

    sensor_state_time = GetTimestamp();
    sensor.getState(sensor_state);
    DumpState("getState", sensor_state);
    LOG("Saved state to RTC memory at %lld", sensor_state_time);
    CheckSensor();

    uint64_t time_us = ((sensor.nextCall - GetTimestamp()) * 1000) - esp_timer_get_time();
    LOG("Deep sleep for %llu ms. BSEC next call at %llu ms.", time_us / 1000, sensor.nextCall);
    esp_sleep_enable_timer_wakeup(time_us);
    esp_deep_sleep_start();
  }
}