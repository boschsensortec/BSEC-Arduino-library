/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * @file	bsec.cpp
 * @date	27 May 2022
 * @version	1.4.1492
 *  
 */

#include "bsec.h"

TwoWire* Bsec::wireObj = NULL;
SPIClass* Bsec::spiObj = NULL;

/**
 * @brief Constructor
 */
Bsec::Bsec()
{
    zeroInputs();
	zeroOutputs();
}

/**
 * @brief Function to initialize the BSEC library and the BME68x sensor
 */
void Bsec::begin(bme68x_intf intf, bme68x_read_fptr_t read, bme68x_write_fptr_t write, bme68x_delay_us_fptr_t idleTask, void *intfPtr)
{
	_bme68x.intf = intf;
	_bme68x.read = read;
	_bme68x.write = write;
	_bme68x.delay_us = idleTask;
	_bme68x.intf_ptr = intfPtr;
	_bme68x.amb_temp = 25;

	beginCommon();
}

/**
 * @brief Function to initialize the BSEC library and the BME68x sensor
 */
void Bsec::begin(uint8_t i2cAddr, TwoWire &i2c)
{
	_bme68x.intf_ptr = (void*)(intptr_t)i2cAddr;
	_bme68x.intf = BME68X_I2C_INTF;
	_bme68x.read = Bsec::i2cRead;
	_bme68x.write = Bsec::i2cWrite;
	_bme68x.delay_us = Bsec::delay_us;
	_bme68x.amb_temp = 25;

	Bsec::wireObj = &i2c;
	Bsec::wireObj->begin();
	beginCommon();
}

/**
 * @brief Function to initialize the BSEC library and the BME68X sensor with SPI communication setting
 */
void Bsec::begin(uint8_t chipSelect, SPIClass &spi)
{
	_bme68x.intf_ptr = (void*)(intptr_t)chipSelect;
	_bme68x.intf = BME68X_SPI_INTF;
	_bme68x.read = Bsec::spiRead;
	_bme68x.write = Bsec::spiWrite;
	_bme68x.delay_us = Bsec::delay_us;
	_bme68x.amb_temp = 25;	

	pinMode(chipSelect, OUTPUT);
	digitalWrite(chipSelect, HIGH);
	Bsec::spiObj = &spi;
	Bsec::spiObj->begin();

	return beginCommon();
}

/**
 * @brief Common code for the begin function
 */
void Bsec::beginCommon(void)
{
    zeroInputs();
    zeroOutputs();

    bsecStatus = bsec_init();

	getVersion();
	
	bme68xStatus = bme68x_init(&_bme68x);
}

/**
 * @brief Function that sets the desired sensors and the sample rates
 */
void Bsec::updateSubscription(bsec_virtual_sensor_t sensorList[], uint8_t nSensors, float sampleRate)
{
	bsec_sensor_configuration_t virtualSensors[BSEC_NUMBER_OUTPUTS],
	        sensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;

	for (uint8_t i = 0; i < nSensors; i++) {
		virtualSensors[i].sensor_id = sensorList[i];
		virtualSensors[i].sample_rate = sampleRate;
	}

	bsecStatus = bsec_update_subscription(virtualSensors, nSensors, sensorSettings, &nSensorSettings);
	return;
}

/**
 * @brief Callback from the user to trigger reading of data from the BME68x, process and store outputs
 */
bool Bsec::run(void)
{
	bool newData = false;
	/* Check if the time has arrived to call do_steps() */
	int64_t callTimeMs = getTimeMs();
	
	if (callTimeMs >= nextCall) {
	
		bsec_bme_settings_t bme68xSettings;

		int64_t callTimeNs = callTimeMs * INT64_C(1000000);

		bsecStatus = bsec_sensor_control(callTimeNs, &bme68xSettings);
		if (bsecStatus < BSEC_OK)
			return false;

		nextCall = bme68xSettings.next_call / INT64_C(1000000); // Convert from ns to ms

		bme68xStatus = setBme68xConfig(bme68xSettings);
		if (bme68xStatus != BME68X_OK) {
			return false;
		}

		newData = readProcessData(callTimeNs, bme68xSettings);	
	}
	
	return newData;
}

/**
 * @brief Function to get the state of the algorithm to save to non-volatile memory
 */
void Bsec::getState(uint8_t *state)
{
	uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];
	uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
	bsecStatus = bsec_get_state(0, state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_STATE_BLOB_SIZE, &n_serialized_state);
}

/**
 * @brief Function to set the state of the algorithm from non-volatile memory
 */
void Bsec::setState(uint8_t *state)
{
	uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];

	bsecStatus = bsec_set_state(state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_STATE_BLOB_SIZE);
}

/**
 * @brief Function to set the configuration of the algorithm from memory
 */
void Bsec::setConfig(const uint8_t *state)
{
	uint8_t workBuffer[BSEC_MAX_PROPERTY_BLOB_SIZE];

	bsecStatus = bsec_set_configuration(state, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, sizeof(workBuffer));
}

/* Private functions */

/**
 * @brief Get the version of the BSEC library
 */
void Bsec::getVersion(void)
{
	bsec_get_version(&version);
}

/**
 * @brief Read data from the BME68x and process it
 */
bool Bsec::readProcessData(int64_t currTimeNs, bsec_bme_settings_t bme68xSettings)
{
	bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; // Temperature, Pressure, Humidity & Gas Resistance
	uint8_t nInputs = 0, nOutputs = 0;
	nFields = 0;

	if (bme68xSettings.process_data)
	{
		bme68xStatus = bme68x_get_data(BME68X_FORCED_MODE, &_data, &nFields, &_bme68x);
		if (bme68xStatus != BME68X_OK)
		{
			return false;
		}
	
		if (nFields)
		{
			if (bme68xSettings.process_data & BSEC_PROCESS_TEMPERATURE)
			{
				inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
				inputs[nInputs].signal = _data.temperature;

				inputs[nInputs].time_stamp = currTimeNs;
				nInputs++;
				/* Temperature offset from the real temperature due to external heat sources */
				inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
				inputs[nInputs].signal = _tempOffset;
				inputs[nInputs].time_stamp = currTimeNs;
				nInputs++;
			}
			if (bme68xSettings.process_data & BSEC_PROCESS_HUMIDITY)
			{
				inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;

				inputs[nInputs].signal = _data.humidity;

				inputs[nInputs].time_stamp = currTimeNs;
				nInputs++;
			}
			if (bme68xSettings.process_data & BSEC_PROCESS_PRESSURE)
			{
				inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
				inputs[nInputs].signal = _data.pressure;
				inputs[nInputs].time_stamp = currTimeNs;
				nInputs++;
			}
			if (bme68xSettings.process_data & BSEC_PROCESS_GAS)
			{
				/* Check whether gas_valid flag is set */
				if (_data.status & BME68X_GASM_VALID_MSK)
				{
					inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
					inputs[nInputs].signal = _data.gas_resistance;
					inputs[nInputs].time_stamp = currTimeNs;
					nInputs++;
				}
			}
		}
	}

	if (nInputs > 0)
	{
		nOutputs = BSEC_NUMBER_OUTPUTS;
		bsec_output_t _outputs[BSEC_NUMBER_OUTPUTS];

		bsecStatus = bsec_do_steps(inputs, nInputs, _outputs, &nOutputs);
		if (bsecStatus != BSEC_OK)
			return false;

		zeroOutputs();

		if (nOutputs > 0)
		{
			outputTimestamp = _outputs[0].time_stamp / 1000000; // Convert from ns to ms

			for (uint8_t i = 0; i < nOutputs; i++)
			{
				switch (_outputs[i].sensor_id)
				{
					case BSEC_OUTPUT_IAQ:
						iaq = _outputs[i].signal;
						iaqAccuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_STATIC_IAQ:
						staticIaq = _outputs[i].signal;
						staticIaqAccuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_CO2_EQUIVALENT:
						co2Equivalent = _outputs[i].signal;
						co2Accuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
						breathVocEquivalent = _outputs[i].signal;
						breathVocAccuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_RAW_TEMPERATURE:
						rawTemperature = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RAW_PRESSURE:
						pressure = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RAW_HUMIDITY:
						rawHumidity = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RAW_GAS:
						gasResistance = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_STABILIZATION_STATUS:
						stabStatus = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RUN_IN_STATUS:
						runInStatus = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
						temperature = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
						humidity = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_GAS_PERCENTAGE:
						gasPercentage = _outputs[i].signal;
						gasPercentageAccuracy = _outputs[i].accuracy;
						break;
					default:
						continue;
				}
			}
			return true;
		}
	}

	return false;
}

/**
 * @brief Set the BME68x sensor's configuration
 */
int8_t Bsec::setBme68xConfig(bsec_bme_settings_t bme68xSettings)
{
	int8_t bme68xSts = BME68X_OK;
	uint16_t meas_period;
	uint8_t current_op_mode;
	
	/* Check if a forced-mode measurement should be triggered now */
    if (bme68xSettings.trigger_measurement)
	{
		conf.filter = BME68X_FILTER_OFF;
		conf.odr = BME68X_ODR_NONE;
		conf.os_hum = bme68xSettings.humidity_oversampling;
		conf.os_temp = bme68xSettings.temperature_oversampling;
		conf.os_pres = bme68xSettings.pressure_oversampling;

		bme68xSts = bme68x_set_conf(&conf, &_bme68x);

		if (bme68xSts == BME68X_ERROR)
		{
			return bme68xSts;	
		}

		heatrConf.enable = bme68xSettings.run_gas;
		heatrConf.heatr_temp = bme68xSettings.heater_temperature;
		heatrConf.heatr_dur = bme68xSettings.heating_duration;

		bme68xSts = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatrConf, &_bme68x);
		
		if (bme68xSts == BME68X_ERROR)
		{
			return bme68xSts;
		}
		
		bme68xSts = bme68x_set_op_mode(BME68X_FORCED_MODE, &_bme68x);

		if (bme68xSts == BME68X_ERROR)
		{
			return bme68xSts;
		}
		meas_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &_bme68x) + heatrConf.heatr_dur*1000;
		/* Delay till the measurement is ready. Timestamp resolution in ms */
        delay_us((uint32_t)meas_period, _bme68x.intf_ptr);
	}

	/* Call the API to get current operation mode of the sensor */
    bme68xSts = bme68x_get_op_mode(&current_op_mode, &_bme68x);
    /* When the measurement is completed and data is ready for reading, the sensor must be in bme68x_SLEEP_MODE.
     * Read operation mode to check whether measurement is completely done and wait until the sensor is no more
     * in bme68x_FORCED_MODE. */
    while (current_op_mode == BME68X_FORCED_MODE)
    {
        /* sleep for 5 ms */
        delay_us(5 * 1000, _bme68x.intf_ptr);
        bme68xSts = bme68x_get_op_mode(&current_op_mode, &_bme68x);
    }
	return bme68xSts;
}

/**
 * @brief Function to zero the outputs
 */
void Bsec::zeroOutputs(void)
{
	temperature = 0.0f;
	pressure = 0.0f;
	humidity = 0.0f;
	gasResistance = 0.0f;
	rawTemperature = 0.0f;
	rawHumidity = 0.0f;
	stabStatus = 0.0f;
	runInStatus = 0.0f;
	iaq = 0.0f;
	iaqAccuracy = 0;
	staticIaq = 0.0f;
	staticIaqAccuracy = 0;
	co2Equivalent = 0.0f;
	co2Accuracy = 0;
	breathVocEquivalent = 0.0f;
	breathVocAccuracy = 0;
	compGasValue = 0.0f;
	compGasAccuracy = 0;
	gasPercentage = 0.0f;
	gasPercentageAccuracy = 0;
}

/**
 * @brief Function to zero the outputs
 */
void Bsec::zeroInputs(void)
{
    nextCall = 0;
    version.major = 0;
    version.minor = 0;
    version.major_bugfix = 0;
    version.minor_bugfix = 0;
    millisOverflowCounter = 0;
    lastTime = 0;
    bme68xStatus = BME68X_OK;
    outputTimestamp = 0;
    _tempOffset = 0.0f;
   	bsecStatus = BSEC_OK;

}
/**
 * @brief Function to calculate an int64_t timestamp in milliseconds
 */
int64_t Bsec::getTimeMs(void)
{
	int64_t timeMs = millis();

	if (lastTime > timeMs) { // An overflow occurred
		millisOverflowCounter++;
	}

	lastTime = timeMs;

	return timeMs + ((int64_t)millisOverflowCounter << 32);
}

/**
 @brief Task that delays for a us period of time
 */
void Bsec::delay_us(uint32_t period, void *intfPtr)
{
	(void) intfPtr;
	// Wait for a period amount of ms
	// The system may simply idle, sleep or even perform background tasks
	delay(period/1000);
}

/**
 @brief Callback function for reading registers over I2C
 */
int8_t Bsec::i2cRead(uint8_t regAddr, uint8_t *regData, uint32_t length, void *intf_ptr)
{
	intptr_t devId = (intptr_t)intf_ptr;
	int8_t rslt = 0;
	if(Bsec::wireObj) {
		Bsec::wireObj->beginTransmission(devId);
		Bsec::wireObj->write(regAddr);
		rslt = Bsec::wireObj->endTransmission();
		Bsec::wireObj->requestFrom((int) devId, (int) length);
		for (auto i = 0; (i < length) && Bsec::wireObj->available(); i++) {
			regData[i] = Bsec::wireObj->read();
		}
	} else {
		rslt = -1;
	}
	return rslt;
}

/**
 * @brief Callback function for writing registers over I2C
 */
int8_t Bsec::i2cWrite(uint8_t regAddr, const uint8_t *regData, uint32_t length, void *intf_ptr)
{
	intptr_t devId = (intptr_t)intf_ptr;
	int8_t rslt = 0;
	if(Bsec::wireObj) {
		Bsec::wireObj->beginTransmission(devId);
		Bsec::wireObj->write(regAddr);
		for (auto i = 0; i < length; i++) {
			Bsec::wireObj->write(regData[i]);
		}
		rslt = Bsec::wireObj->endTransmission();
	} else {
		rslt = -1;
	}
	return rslt;
}

/**
 * @brief Callback function for reading over SPI
 */
int8_t Bsec::spiRead(uint8_t regAddr, uint8_t *regData, uint32_t length, void *intf_ptr)
{
	intptr_t devId = (intptr_t)intf_ptr;
	int8_t rslt = 0;
	if(Bsec::spiObj) {
		Bsec::spiObj->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // Can be up to 10MHz

		digitalWrite(devId, LOW);

		Bsec::spiObj->transfer(regAddr); // Write the register address, ignore the return
		for (auto i = 0; i < length; i++)
			regData[i] = Bsec::spiObj->transfer(regData[i]);

		digitalWrite(devId, HIGH);
		Bsec::spiObj->endTransaction();
	} else {
		rslt = -1;
	}

	return rslt;
}

/**
 * @brief Callback function for writing registers over SPI
 */
int8_t Bsec::spiWrite(uint8_t regAddr, const uint8_t *regData, uint32_t length, void *intf_ptr)
{
	intptr_t devId = (intptr_t)intf_ptr;
	int8_t rslt = 0;
	if(Bsec::spiObj) {
		Bsec::spiObj->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // Can be up to 10MHz

		digitalWrite(devId, LOW);

		Bsec::spiObj->transfer(regAddr); // Write the register address, ignore the return
		for (auto i = 0; i < length; i++)
			Bsec::spiObj->transfer(regData[i]);

		digitalWrite(devId, HIGH);
		Bsec::spiObj->endTransaction();
	} else {
		rslt = -1;
	}

	return rslt;
}

/**
 * @brief Function to set the Temperature, Pressure and Humidity over-sampling
 */
void Bsec::setTPH(uint8_t osTemp, uint8_t osPres, uint8_t osHum)
{
	
}
