#include "sensor.h"

static const uint8_t temp_cmd[2]  = { SEESAW_STATUS_BASE, SEESAW_STATUS_TEMP };
static const uint8_t touch_cmd[2] = { SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET };

volatile SensorState_t sensorState = SENSOR_STATE_IDLE;
volatile uint32_t nextMeasurementTick = 0;

static uint8_t temp_data[4];
static uint8_t touch_data[2];

static float measured_temperature = 0.0f;
static uint16_t measured_moisture = 0;

extern I2C_HandleTypeDef hi2c1;

volatile uint8_t sensor_rx_done = 0;
volatile uint8_t sensor_tx_done = 0;
volatile uint8_t sensor_error_flag = 0;

extern volatile SensorMeasurment_t BUF_SENSOR[SENSOR_BUF_LEN];
extern volatile uint16_t empty_SENSOR;
extern volatile uint16_t busy_SENSOR;

void sensor_process() {
	if(sensor_error_flag) {
		sensor_error_flag = 0;
		sensorState = SENSOR_STATE_IDLE;
		nextMeasurementTick = HAL_GetTick() + read_interval;
	}

	switch(sensorState) {
	case SENSOR_STATE_IDLE:
		// Interval disabled
		if (read_interval == 0) {
			return;
		}

	    if(HAL_GetTick() >= nextMeasurementTick) {
	        sensorState = SENSOR_STATE_TEMP_WRITE;
	        if(HAL_I2C_Master_Transmit_IT(&hi2c1, SENSOR_I2C_ADDR, (uint8_t*)temp_cmd, 2) != HAL_OK) {
	            sensorState = SENSOR_STATE_IDLE;
	            nextMeasurementTick = HAL_GetTick() + read_interval;
	        }
	    }
	    break;

	case SENSOR_STATE_TEMP_WRITE:
	    if(sensor_tx_done) {
	        sensor_tx_done = 0;
	        sensorState = SENSOR_STATE_TEMP_READ;
	        if(HAL_I2C_Master_Receive_IT(&hi2c1, SENSOR_I2C_ADDR, temp_data, 4) != HAL_OK) {
	            sensorState = SENSOR_STATE_IDLE;
	            nextMeasurementTick = HAL_GetTick() + read_interval;
	        }
	    }
	    break;

	case SENSOR_STATE_TEMP_READ:
	    if(sensor_rx_done) {
	        sensor_rx_done = 0;
	        uint32_t temp_raw = ((uint32_t)temp_data[0] << 24) |
	                            ((uint32_t)temp_data[1] << 16) |
	                            ((uint32_t)temp_data[2] << 8)  |
	                            (uint32_t)temp_data[3];
	        measured_temperature = (float)temp_raw / 65536.0f;
	        sensorState = SENSOR_STATE_TOUCH_WRITE;
	        if(HAL_I2C_Master_Transmit_IT(&hi2c1, SENSOR_I2C_ADDR, (uint8_t *)touch_cmd, 2) != HAL_OK) {
	            sensorState = SENSOR_STATE_IDLE;
	            nextMeasurementTick = HAL_GetTick() + read_interval;
	        }
	    }
	    break;

	case SENSOR_STATE_TOUCH_WRITE:
	    if(sensor_tx_done) {
	        sensor_tx_done = 0;
	        sensorState = SENSOR_STATE_TOUCH_READ;
	        if(HAL_I2C_Master_Receive_IT(&hi2c1, SENSOR_I2C_ADDR, touch_data, 2) != HAL_OK) {
	            sensorState = SENSOR_STATE_IDLE;
	            nextMeasurementTick = HAL_GetTick() + read_interval;
	        }
	    }
	    break;

	case SENSOR_STATE_TOUCH_READ:
	    if(sensor_rx_done) {
	        sensor_rx_done = 0;
	        uint16_t raw_moisture = ((uint16_t)touch_data[0] << 8) | touch_data[1];
	        // Skalowanie liniowe: ((x - 200) / (1800)) * 100
	        measured_moisture = (((uint32_t)(raw_moisture - 200)) * 100) / 1800;
	        sensorState = SENSOR_STATE_PROCESS;
	    }
	    break;

	case SENSOR_STATE_PROCESS:
	    // Zapisuje dane w buforze
	    BUF_SENSOR[empty_SENSOR].temperature = measured_temperature;
	    BUF_SENSOR[empty_SENSOR].moisture = measured_moisture;
	    {
	        uint16_t nextIndex = (empty_SENSOR + 1) % SENSOR_BUF_LEN;
	        if(nextIndex == busy_SENSOR) {
	            busy_SENSOR = (busy_SENSOR + 1) % SENSOR_BUF_LEN;
	        }
	        empty_SENSOR = nextIndex;
	    }
	    // Ustawia opóźnienie przed wysłaniem do konsoli
	    nextMeasurementTick = HAL_GetTick() + read_interval;
	    sensorState = SENSOR_STATE_DELAY;
	    break;

	case SENSOR_STATE_DELAY:
		// Interval disabled
		if (read_interval == 0) {
			sensorState = SENSOR_STATE_IDLE;
		    return;
		}
	    // Czeka aż upłynie zadany interwał
	    if(HAL_GetTick() >= nextMeasurementTick) {
	        sensor_log();  //DEBUG -  wysyła dane do konsoli
	        sensorState = SENSOR_STATE_IDLE;
	        nextMeasurementTick = HAL_GetTick() + read_interval;
	    }
	    break;

	default:
	    sensorState = SENSOR_STATE_IDLE;
	    break;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c->Instance == hi2c1.Instance) {
		sensor_tx_done = 1;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c->Instance == hi2c1.Instance) {
		sensor_rx_done = 1;
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c->Instance == hi2c1.Instance) {
		sensor_error_flag = 1;
	}
}
