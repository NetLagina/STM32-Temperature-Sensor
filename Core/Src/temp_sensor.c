/*
 * temp_sensor.c
 *
 *  Created on: Aug 4, 2019
 *      Author: NetL
 */

#include <main.h>

#include <temp_sensor.h>

const uint32_t MAX_TIMER = 20000;

void temperature_sensor_init() {
	HAL_Delay(2000);
}

void GPIO_WritePin(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, PinState);
}

GPIO_PinState GPIO_ReadPin(void) {
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
}

void set_GPIO_input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void set_GPIO_output(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void delay(uint32_t delay) {
	for (int i = 0; i < delay; ++i) {
	}
}

TemperatureSensor get_temperature_sensor_data(TemperatureSensor ts) {
	ts.is_valid = 0;
	set_GPIO_output();
	GPIO_WritePin(GPIO_PIN_RESET);
	HAL_Delay(10);
	GPIO_WritePin(GPIO_PIN_SET);
	set_GPIO_input();

	uint32_t timer = 0;
	while (GPIO_ReadPin() != GPIO_PIN_RESET && timer < MAX_TIMER) {
		++timer;
	}
	timer = 0;
	while (GPIO_ReadPin() != GPIO_PIN_SET && timer < MAX_TIMER) {
		++timer;
	}
	timer = 0;
	while (GPIO_ReadPin() != GPIO_PIN_RESET && timer < MAX_TIMER) {
		++timer;
	}

	if (timer == MAX_TIMER) {
		return ts; // time out
	}

	uint8_t data[5] = { 0b0, 0b0, 0b0, 0b0, 0b0 };

	int i = 0;
	int j = 7;
	while (i != 5) {
		while (GPIO_ReadPin() != GPIO_PIN_SET) {
			//wait
		}
		delay(37);
		if (GPIO_ReadPin() == GPIO_PIN_SET) {
			data[i] |= 1 << j;
		}
		if (!j) {
			++i;
			j = 8;
		}
		--j;
	}

	uint16_t rh = (data[0] << 8) + data[1];
	int16_t t = (data[2] << 8) + data[3];
	uint8_t sum = data[0] + data[1] + data[2] + data[3];
	if (sum == data[4]) {
		ts.relative_humidity = rh;
		ts.temp = t;
		ts.is_valid = 1;
	}
	return ts;
}

int16_t get_temperature_sensor_temperature(TemperatureSensor ts) {
	return ts.temp;
}

int16_t get_temperature_sensor_rel_humidity(TemperatureSensor ts) {
	return ts.relative_humidity;
}
