/*
 * tensometer.h
 *
 *  Created on: 16.05.2018
 *      Author: Norbi
 */

#ifndef TENSOMETER_H_
#define TENSOMETER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct _hx711
{
	GPIO_TypeDef* gpioSck;
	GPIO_TypeDef* gpioData;
	uint16_t pinSck;
	uint16_t pinData;
	int offset;
	int gain;
	  // 1: channel A, gain factor 128
	  // 2: channel B, gain factor 32
    // 3: channel A, gain factor 64
} HX711;


void HX711_Init(HX711 data);
HX711 HX711_Tare(HX711 data, uint8_t times);
int32_t HX711_Value(HX711 data);
int32_t HX711_AverageValue(HX711 data, uint8_t times);
int32_t convertRAWtensometer(HX711 data);
void tensometerSensorLogger(void);
void tensometerTest(void);
void tensometerSensorLoggerRaw(void);

void cliGetTensoSample(uint32_t argc, char *argv[]);
void cliCalibrateTenso(uint32_t argc, char *argv[]);


#endif /* TENSOMETER_H_ */
