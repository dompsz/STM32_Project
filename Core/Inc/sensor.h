#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SENSOR_I2C_ADDR (0x36 << 1)  // Adres I2C czujnika (przesunięty o 1 bit, tak wymaga HAL)
#define SENSOR_BUF_LEN 600

#define SEESAW_STATUS_BASE 0x00
#define SEESAW_STATUS_TEMP 0x04

#define SEESAW_TOUCH_BASE 0x0F
#define SEESAW_TOUCH_CHANNEL_OFFSET 0x10  // dla kanału 0

typedef enum {
  SENSOR_STATE_IDLE = 0,
  SENSOR_STATE_TEMP_WRITE,
  SENSOR_STATE_TEMP_READ,
  SENSOR_STATE_TOUCH_WRITE,
  SENSOR_STATE_TOUCH_READ,
  SENSOR_STATE_DELAY,
  SENSOR_STATE_PROCESS
} SensorState_t;

typedef struct {
	float temperature;
	uint16_t moisture;
} SensorMeasurment_t;

extern volatile uint32_t read_interval;

void sensor_process();

#endif  // SENSOR_H
