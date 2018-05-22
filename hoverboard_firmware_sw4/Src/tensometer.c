#include <stdlib.h>
#include "Tensometer.h"
#include "main.h"

#define TENSO_SAMPLE_COUNT	4

typedef struct
{
	int32_t mbar;
	int32_t tensoraw;
} STensoSample;



//using HX711 module

int32_t tensometer;
int32_t raw_tens;



	int32_t time = 0;
	
	int32_t up = 0;
	int32_t down = 0;
	int32_t raw_up = 0;
	int32_t raw_down = 0;
	
//	int32_t tensometer = 0;
//	int32_t raw_tens = 0;
	
	int32_t last_tensometer = 0;
	int32_t last_raw_tens = 0;
	uint16_t size = 0; // Rozmiar wysylanej wiadomosci


HX711 dataHX = {HX_SCK_GPIO_Port, HX_DATA_GPIO_Port, HX_SCK_Pin, HX_DATA_Pin, 0, 3};

void HX711_Init(HX711 data)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = data.pinSck;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioSck, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = data.pinData;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioData, &GPIO_InitStruct);

	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
	
}

int32_t HX711_Average_Value(HX711 data, uint8_t times)
{
    int sum = 0;
    for (int i = 0; i < times; i++)
    {
        sum += HX711_Value(data);
    }

    return sum / times;
}

int32_t HX711_Value(HX711 data)
{
		__disable_irq();
	
    int32_t buffer;
    buffer = 0;
	
	while (HAL_GPIO_ReadPin(data.gpioData, data.pinData)==1);
		
    for (uint8_t i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(data.gpioData, data.pinData))
        {
            buffer ++;
        }

        HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    }

    for (int32_t i = 0; i < data.gain; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    }

    buffer = buffer ^ 0x800000;
		
		__enable_irq();
    return buffer;
}

HX711 HX711_Tare(HX711 data, uint8_t times)
{
    int32_t sum = HX711_Average_Value(data, times);
    data.offset = sum;
    return data;
}

int32_t convertRAWtensometer(HX711 data) {
//	int32_t tens;
	double tens = 0;

	
	double a = 0;
	double b = 0;

	tens = HX711_Value(data);
//	tens = HX711_Average_Value(data,5);
	raw_tens = tens;

	tens = a*tens+b;

	return tens;
}

void tensometerSensorLogger(void){
	

	
		last_tensometer = tensometer;
		tensometer = convertRAWtensometer(dataHX);
	
		char buff[64];
		sprintf(buff, "TEN:%d", tensometer);
	//	zuart_send_string(buff);
	
		if (tensometer > last_tensometer+20)
				up++;
		
		if (tensometer < last_tensometer-20)
				down++;


}


void tensometerTest(void){
		tensometer = convertRAWtensometer(dataHX);
		
		char buff[64];
		sprintf(buff, "TEN:%d", tensometer);
//		zuart_send_string(buff);
	
		last_raw_tens = raw_tens;
		raw_tens = HX711_Value(dataHX);
		
		if (raw_tens > last_raw_tens+50000)
				raw_up++;
		
		if (raw_tens < last_raw_tens-50000)
				raw_down++;
		
		last_tensometer = tensometer;
		tensometer = convertRAWtensometer(dataHX);
		if (tensometer > last_tensometer+50)
				up++;
		
		if (tensometer < last_tensometer-50)
				down++;

		time++;	

	
}
