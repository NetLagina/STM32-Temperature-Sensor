/*
 * temp_sensor.h
 *
 *  Created on: Aug 4, 2019
 *      Author: NetL
 */

#ifndef TEMP_SENSOR_H_
#define TEMP_SENSOR_H_

#include "stm32f1xx.h"

typedef struct {
	uint16_t relative_humidity;
	uint16_t temp;
	uint8_t is_valid;
} TemperatureSensor;

void temperature_sensor_init();
TemperatureSensor get_temperature_sensor_data(TemperatureSensor);
int16_t get_temperature_sensor_temperature(TemperatureSensor);
int16_t get_temperature_sensor_rel_humidity(TemperatureSensor);

#endif /* TEMP_SENSOR_H_ */
