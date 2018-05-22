/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include "BLDC_Motors.h"
#include "tm_stm32_mpu6050.h"
#include "tm_stm32_delay.h"
#include "matrix.h"
#include "kalman_filter.h"
#include "PID_regulator.h"
#include "tensometer.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t allow = 0;
uint8_t Received[4] = "0";
uint8_t receive;
uint8_t rec_i = 0;
uint8_t temp_char[3];

//////////////////////Temporary
float kal_result = 0;
float angle = 0;		//before kalman
int32_t d_kal_result = 0;
int32_t d_angle = 0;		//before kalman

int16_t pid = 0;
uint32_t tenso = 0;
uint32_t co = 0;

TM_MPU6050_t MPU6050_Data0;


//temporary for tuning PID and Kalman
uint8_t Kp_t;                 // (P)roportional Tuning Parameter
uint8_t Ki_t;					// (I)ntegral Tuning Parameter
uint8_t Kd_t;					// (D)erivative Tuning Parameter

//uint8_t std_dev_v_t = std_dev_v;
//uint8_t std_dev_w_t = std_dev_w;
//Temporary

////////////////////////Temporary

//temporary for tuning PID and Kalman
extern float Kp;                 // (P)roportional Tuning Parameter
extern float Ki;					// (I)ntegral Tuning Parameter
extern float Kd;					// (D)erivative Tuning Parameter

extern float std_dev_v;
extern float std_dev_w;
extern uint8_t lastOut;
//Temporary

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM12_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if(htim->Instance == TIM6){ // Je�eli przerwanie pochodzi od timera 6


	 for(rec_i = 0;rec_i<3;++rec_i)
		 	 temp_char[rec_i] = Received[rec_i+1];

//	 Kp = atoi(temp_char);
	 if (Received[0] == 80 || Received[0] == 112)	//P
		 Kp = atoi(temp_char);
	 if (Received[0] == 73 || Received[0] == 105)	//I
		 Ki = atoi(temp_char);
	 if (Received[0] == 68 || Received[0] == 100)	//D
		 Kd = atoi(temp_char);

	 //Temporary
	 Kp_t = Kp;                // (P)roportional Tuning Parameter
	 Ki_t = Ki;					// (I)ntegral Tuning Parameter
	 Kd_t = Kd;					// (D)erivative Tuning Parameter
//	 receive = atoi(Received);
//	 Ki = receive;


	 uint8_t str[50];
	 uint16_t size;
 //	  size = sprintf(str, "%d\r\n", HAL_GetTick());
 //	  HAL_UART_Transmit_IT(&huart3, str, size);


 	  TM_MPU6050_ReadAll(&MPU6050_Data0);
	  kal_result = kalman_filter_get_est(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_Y);
	  d_kal_result = kal_result;

	  angle = angle_before_kalman(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z);
	  d_angle = angle;

	  pid = PID_calculate(-1,kal_result);
	  Control_Motor_by_PID(BOTH_MOTORS, pid);

  	  size = sprintf(str, "%d %d %d  %d  %d %d %d %d %d\r\n", Kp_t, Ki_t, Kd_t, pid, d_angle, d_kal_result, -1000, 1000, lastOut);
  	  HAL_UART_Transmit_IT(&huart3, str, size);

	 /*
	 uint8_t str[50];
	 uint16_t size;
 	  size = sprintf(str, "%d\r\n", HAL_GetTick());
 	  HAL_UART_Transmit_IT(&huart3, str, size);
*/
//	 allow=1;
 }



}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

 uint8_t Data[40]; // Tablica przechowujaca wysylana wiadomosc.
 uint16_t size = 0; // Rozmiar wysylanej wiadomosci

// size = sprintf(Data, "Odebrana wiadomosc: %s\n\r",Received);

 size = sprintf(Data, "%s\n\r",Received);

 HAL_UART_Transmit_IT(&huart3, Data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
 HAL_UART_Receive_IT(&huart3, Received, 1); // Ponowne w��czenie nas�uchiwania
}
*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /*
//  __disable_irq();
  HX711 hx711 = {HX_SCK_GPIO_Port, HX_DATA_GPIO_Port, HX_SCK_Pin, HX_DATA_Pin, 0, 1};
  hx711 = HX711_Tare(hx711, 10);
  HX711_Init(hx711);
//  __enable_irq();
*/

  //uart example
  uint8_t i = 9;
  uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
  uint16_t size = 0; // Rozmiar wysylanej wiadomosci
  size = sprintf(data, "kutaczan %d\r\n", i); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.


  //BLDC_Motors Init
  Initialize_LR_Motors(&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2);
  Stop_LR_Motors();


  //MPU6050 Init

 // TM_MPU6050_t MPU6050_Data0;
  TM_MPU6050_t MPU6050_Data1;
  uint8_t sensor1 = 0, sensor2 = 0;
  char str[120];

  // Initialize MPU6050 sensor 0, address = 0xD0, AD0 pin on sensor is low
  if (TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok) {
          // Display message to user
  //        TM_USART_Puts(USART1, "MPU6050 sensor 0 is ready to use!\n");
          size = sprintf(str, "MPU6050 sensor 0 is ready to use!\n", i);
  //  	  HAL_UART_Transmit(&huart3, str, size, 1000);
          // Sensor 1 OK
          sensor1 = 1;
  }

  // Initialize MPU6050 sensor 1, address = 0xD2, AD0 pin on sensor is high
  if (TM_MPU6050_Init(&MPU6050_Data1, TM_MPU6050_Device_1, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok) {
        // Display message to user
    //    TM_USART_Puts(USART1, "MPU6050 sensor 1 is ready to use!\n");
        size = sprintf(str, "MPU6050 sensor 1 is ready to use!\n", i);
 // 	  HAL_UART_Transmit(&huart3, str, size, 1000);
        // Sensor 2 OK
        sensor2 = 1;
  }







  //Kalman filter init
  TM_MPU6050_ReadAll(&MPU6050_Data0);
  kalman_filter_init(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z);


  //Init timer od wywo�ania pomiaru i PID
  HAL_TIM_Base_Start_IT(&htim6);


  int8_t last_v = 0;
    /*
    //program do wyznaczenia transmitancji

    HAL_Delay(1000);
    for (uint8_t i = 0;i<200;i++){

    	HAL_Delay(5);
  		TM_MPU6050_ReadAll(&MPU6050_Data0);
		kal_result = kalman_filter_get_est(MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_Z);
		d_kal_result = kal_result;

		angle = angle_before_kalman(MPU6050_Data0.Accelerometer_Y, MPU6050_Data0.Accelerometer_Z);
		d_angle = angle;
		size = sprintf(str, "%d %d\r\n", d_angle, d_kal_result);
		HAL_UART_Transmit_IT(&huart3, str, size);

  	  if (i==10)
  	  {
  		  Set_Left_Motor_DIR(FORWARD);
  		  Set_Right_Motor_DIR(FORWARD);
  		  Set_Left_Motor_Speed(20);
  		  Set_Right_Motor_Speed(20);
  	  }

  	  if (i>198)
  	  {
 // 		Set_Left_Motor_Speed(0);
  //		Set_Right_Motor_Speed(0);
//  		Set_LR_Motors_Speed(0,0);
  		Stop_LR_Motors();
  	  }

  	}

*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


/*
 Struktura docelowa g��wnej p�tli:
 Pomiar MPU 1 i 2
 Filtracja 1 i 2 kalmanem
 Obliczenie PID�w dla 1 i 2
 Wysterowanie silnik�w L i R za pomoc� warto�ci PID�w 1 i 2
 */

	  /*
	  //Pomiar tensometryczny
	  co++;
	  tenso = HX711_Value(hx711);
  	  size = sprintf(str, "%d %d\r\n", tenso, co);
  	  HAL_UART_Transmit_IT(&huart3, str, size);
  	  HAL_Delay(200);
	  */


  	  HAL_UART_Receive_IT(&huart3, Received, 4);

/*
	  if (allow ==1){
		 //Temporary
		 Kp_t = Kp;                // (P)roportional Tuning Parameter
		 Ki_t = Ki;					// (I)ntegral Tuning Parameter
		 Kd_t = Kd;					// (D)erivative Tuning Parameter
	  //Ma�y segway

	  	  TM_MPU6050_ReadAll(&MPU6050_Data0);
	  	  kal_result = kalman_filter_get_est(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_Y);
	  	  d_kal_result = kal_result;

		  angle = angle_before_kalman(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z);
		  d_angle = angle;

	  	  pid = PID_calculate(0,kal_result);
	  	  Control_Motor_by_PID(BOTH_MOTORS, pid);

	  	  size = sprintf(str, "%d %d %d  %d  %d %d %d %d %d\r\n", Kp_t, Ki_t, Kd_t, pid, d_angle, d_kal_result, -100, 100, HAL_GetTick());
	  	  HAL_UART_Transmit_IT(&huart3, str, size);
		  allow=0;
	   }
*/


	//  Control_Motor_by_PID(BOTH_MOTORS, 60);


/*
	  	  // obliczanie k�ta i filtracja kalmanem
	       if (TM_DELAY_Time() >= 5) {
	           TM_DELAY_SetTime(0);
               TM_MPU6050_ReadAll(&MPU6050_Data0);


			   kal_result = kalman_filter_get_est(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z, MPU6050_Data0.Gyroscope_Y);
			   d_kal_result = kal_result;

			   angle = angle_before_kalman(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z);
			   d_angle = angle;
			   size = sprintf(str, "%d %d\r\n", d_angle, d_kal_result);
	//		   Set_Left_Motor_Speed(abs(d_kal_result));
	//		   Set_Right_Motor_Speed(abs(d_angle));
			   HAL_UART_Transmit_IT(&huart3, str, size);
	       }

*/



/*
      //Every 500ms
       if (TM_DELAY_Time() >= 500) {
           // Reset time
           TM_DELAY_SetTime(0);

           //vIf sensor 0 is connected
           if (sensor1) {
               // Read all data from sensor 0
               TM_MPU6050_ReadAll(&MPU6050_Data0);

               // Format data
               size = sprintf(str, "\r\n R \r\n 0 Acc: X:%d   Y:%d   Z:%d \r\n Gyr: X:%d   Y:%d   Z:%d \r\n Temp: %3.4f\r\n",
                   MPU6050_Data0.Accelerometer_X,
                   MPU6050_Data0.Accelerometer_Y,
                   MPU6050_Data0.Accelerometer_Z,
                   MPU6050_Data0.Gyroscope_X,
                   MPU6050_Data0.Gyroscope_Y,
                   MPU6050_Data0.Gyroscope_Z,
                   MPU6050_Data0.Temperature
               );

               // Show to usart
      //         TM_USART_Puts(USART1, str);
               HAL_UART_Transmit_IT(&huart3, str, size);
        //       HAL_UART_Transmit(&huart3, str, size, 500);
           }

           // If sensor 1 is connected
           if (sensor2) {
               // Read all data from sensor 1
               TM_MPU6050_ReadAll(&MPU6050_Data1);

               // Format data
               size = sprintf(str, "\r\n R \r\n 1 Acc: X:%d   Y:%d   Z:%d \r\n Gyr: X:%d   Y:%d   Z:%d \r\n Temp: %3.4f\r\n",
                   MPU6050_Data1.Accelerometer_X,
                   MPU6050_Data1.Accelerometer_Y,
                   MPU6050_Data1.Accelerometer_Z,
                   MPU6050_Data1.Gyroscope_X,
                   MPU6050_Data1.Gyroscope_Y,
                   MPU6050_Data1.Gyroscope_Z,
                   MPU6050_Data1.Temperature
               );

               // Show to usart
     //          TM_USART_Puts(USART1, str);
      //         HAL_UART_Transmit_IT(&huart3, str, size);
               HAL_UART_Transmit(&huart3, str, size, 500);
           }
       }

*/

/*
	  TM_MPU6050_ReadAccelerometer(&MPU6050_Data0);

       	   if(MPU6050_Data0.Accelerometer_Y>0)
       	   {
       		   R_MOT_FORWARD;
       		   Set_Right_Motor_Speed(MPU6050_Data0.Accelerometer_Y/40);
       	   } else if(MPU6050_Data0.Accelerometer_Y<0)
       	   {
       		   R_MOT_BACKWARD;
       		   Set_Right_Motor_Speed(MPU6050_Data0.Accelerometer_Y/-40);
       	   }
/*
	   TM_MPU6050_ReadAccelerometer(&MPU6050_Data1);

		   if(MPU6050_Data1.Accelerometer_Y>0)
		   {
			   L_MOT_BACKWARD;
			   Set_Left_Motor_Speed(MPU6050_Data1.Accelerometer_Y/40);
		   } else if(MPU6050_Data1.Accelerometer_Y<0)
		   {
			   L_MOT_FORWARD;
			   Set_Left_Motor_Speed(MPU6050_Data1.Accelerometer_Y/-40);
		   }
*/

 	//  setBLDC_MotorsPower(15, 15);


	  /*
	  HAL_UART_Transmit_IT(&huart3, data, size);
	  HAL_Delay(1000);




	  setBLDC_MotorsDIR(1, 1);
	  setBLDC_MotorsPower(15, 15);
*/


	  /*
//	  setBLDC_MotorsDIR(0, 0);
//	  HAL_Delay(1000);
//	  setBLDC_MotorsDIR(1, 1);
//	  HAL_Delay(1000);

	  setBLDC_MotorsDIR(1, 0);
	  HAL_Delay(1000);

//	  send_string("Right");
	  setBLDC_MotorsDIR(0, 1);
	  */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5-1;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM12 init function */
static void MX_TIM12_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 16;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim12);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins
     PE4   ------> SAI1_FS_A
     PG14   ------> USART6_TX
     PE1   ------> FMC_NBL1
     PE0   ------> FMC_NBL0
     PB3   ------> I2S3_CK
     PC12   ------> SDIO_CK
     PE5   ------> SAI1_SCK_A
     PB6   ------> QUADSPI_BK1_NCS
     PG15   ------> FMC_SDNCAS
     PD6   ------> SAI1_SD_A
     PD0   ------> FMC_D2_DA2
     PC11   ------> SDIO_D3
     PC10   ------> SDIO_D2
     PA12   ------> USB_OTG_FS_DP
     PI4   ------> FMC_NBL2
     PD1   ------> FMC_D3_DA3
     PI3   ------> FMC_D27
     PI2   ------> FMC_D26
     PA11   ------> USB_OTG_FS_DM
     PF0   ------> FMC_A0
     PI5   ------> FMC_NBL3
     PI7   ------> FMC_D29
     PI10   ------> FMC_D31
     PI6   ------> FMC_D28
     PG9   ------> USART6_RX
     PD2   ------> SDIO_CMD
     PH15   ------> FMC_D23
     PI1   ------> FMC_D25
     PA10   ------> USB_OTG_FS_ID
     PF1   ------> FMC_A1
     PI9   ------> FMC_D30
     PH13   ------> FMC_D21
     PH14   ------> FMC_D22
     PI0   ------> FMC_D24
     PA9   ------> USB_OTG_FS_VBUS
     PC9   ------> SDIO_D1
     PF2   ------> FMC_A2
     PC8   ------> SDIO_D0
     PF3   ------> FMC_A3
     PH4   ------> I2C2_SCL
     PG8   ------> FMC_SDCLK
     PF4   ------> FMC_A4
     PH5   ------> I2C2_SDA
     PH3   ------> FMC_SDNE0
     PG7   ------> SAI1_MCLK_A
     PF7   ------> QUADSPI_BK1_IO2
     PF6   ------> QUADSPI_BK1_IO3
     PF5   ------> FMC_A5
     PH2   ------> FMC_SDCKE0
     PD15   ------> FMC_D1_DA1
     PD10   ------> FMC_D15_DA15
     PF10   ------> QUADSPI_CLK
     PF9   ------> QUADSPI_BK1_IO1
     PF8   ------> QUADSPI_BK1_IO0
     PD14   ------> FMC_D0_DA0
     PD9   ------> FMC_D14_DA14
     PD8   ------> FMC_D13_DA13
     PC0   ------> FMC_SDNWE
     PF12   ------> FMC_A6
     PG1   ------> FMC_A11
     PF15   ------> FMC_A9
     PD13   ------> S_TIM4_CH2
     PH12   ------> FMC_D20
     PF13   ------> FMC_A7
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PG5   ------> FMC_A15_BA1
     PG4   ------> FMC_A14_BA0
     PH9   ------> FMC_D17
     PH11   ------> FMC_D19
     PF14   ------> FMC_A8
     PJ2   ------> DSIHOST_TE
     PF11   ------> FMC_SDNRAS
     PE9   ------> FMC_D6_DA6
     PE11   ------> FMC_D8_DA8
     PE14   ------> FMC_D11_DA11
     PH8   ------> FMC_D16
     PH10   ------> FMC_D18
     PE7   ------> FMC_D4_DA4
     PE10   ------> FMC_D7_DA7
     PE12   ------> FMC_D9_DA9
     PE15   ------> FMC_D12_DA12
     PE13   ------> FMC_D10_DA10
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPKR_HP_Pin|AUDIO_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin|RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin|HX_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SAI1_FSA_Pin SAI1_SCKA_Pin */
  GPIO_InitStruct.Pin = SAI1_FSA_Pin|SAI1_SCKA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPKR_HP_Pin AUDIO_RST_Pin */
  GPIO_InitStruct.Pin = SPKR_HP_Pin|AUDIO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_USART6_TX_Pin USART6_RX_Pin */
  GPIO_InitStruct.Pin = ARDUINO_USART6_TX_Pin|USART6_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_NBL1_Pin FMC_NBL0_Pin D5_Pin D6_Pin 
                           D8_Pin D11_Pin D4_Pin D7_Pin 
                           D9_Pin D12_Pin D10_Pin */
  GPIO_InitStruct.Pin = FMC_NBL1_Pin|FMC_NBL0_Pin|D5_Pin|D6_Pin 
                          |D8_Pin|D11_Pin|D4_Pin|D7_Pin 
                          |D9_Pin|D12_Pin|D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_CK_Pin */
  GPIO_InitStruct.Pin = I2S3_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : uSD_CLK_Pin uSD_D3_Pin uSD_D2_Pin uSD_D1_Pin 
                           uSD_D0_Pin */
  GPIO_InitStruct.Pin = uSD_CLK_Pin|uSD_D3_Pin|uSD_D2_Pin|uSD_D1_Pin 
                          |uSD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS1_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS1_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_BK1_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(QSPI_BK1_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDNCAS_Pin SDCLK_Pin A11_Pin A10_Pin 
                           PG5 PG4 */
  GPIO_InitStruct.Pin = SDNCAS_Pin|SDCLK_Pin|A11_Pin|A10_Pin 
                          |GPIO_PIN_5|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_DATA_Pin */
  GPIO_InitStruct.Pin = MIC_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(MIC_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D3_Pin D1_Pin D15_Pin 
                           D0_Pin D14_Pin D13_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D1_Pin|D15_Pin 
                          |D0_Pin|D14_Pin|D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS1_P_Pin USB_FS1_N_Pin USB_FS1_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS1_P_Pin|USB_FS1_N_Pin|USB_FS1_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_NBL2_Pin D27_Pin D26_Pin FMC_NBL3_Pin 
                           D29_Pin D31_Pin D28_Pin D25_Pin 
                           D30_Pin D24_Pin */
  GPIO_InitStruct.Pin = FMC_NBL2_Pin|D27_Pin|D26_Pin|FMC_NBL3_Pin 
                          |D29_Pin|D31_Pin|D28_Pin|D25_Pin 
                          |D30_Pin|D24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : HX_DATA_Pin */
  GPIO_InitStruct.Pin = HX_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HX_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin 
                           A4_Pin A5_Pin A6_Pin A9_Pin 
                           A7_Pin A8_Pin SDNMT48LC4M32B2B5_6A_RAS_RAS___Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin 
                          |A4_Pin|A5_Pin|A6_Pin|A9_Pin 
                          |A7_Pin|A8_Pin|SDNMT48LC4M32B2B5_6A_RAS_RAS___Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_CMD_Pin */
  GPIO_InitStruct.Pin = uSD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(uSD_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D23_Pin D21_Pin D22_Pin SDNE0_Pin 
                           SDCKE0_Pin D20_Pin D17_Pin D19_Pin 
                           D16_Pin D18_Pin */
  GPIO_InitStruct.Pin = D23_Pin|D21_Pin|D22_Pin|SDNE0_Pin 
                          |SDCKE0_Pin|D20_Pin|D17_Pin|D19_Pin 
                          |D16_Pin|D18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS1_Pin */
  GPIO_InitStruct.Pin = VBUS_FS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI1_MCLKA_Pin */
  GPIO_InitStruct.Pin = SAI1_MCLKA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(SAI1_MCLKA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO2_Pin QSPI_BK1_IO3_Pin QSPI_CLK_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO2_Pin|QSPI_BK1_IO3_Pin|QSPI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO1_Pin QSPI_BK1_IO0_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO1_Pin|QSPI_BK1_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_MOTOR_DIR_Pin RIGHT_MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = LEFT_MOTOR_DIR_Pin|RIGHT_MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SDNWE_Pin */
  GPIO_InitStruct.Pin = SDNWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS1_PowerSwitchOn_Pin EXT_RESET_Pin HX_SCK_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin|HX_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CK_Pin */
  GPIO_InitStruct.Pin = MIC_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(MIC_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DSI_TE_Pin */
  GPIO_InitStruct.Pin = DSI_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DSI;
  HAL_GPIO_Init(DSI_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
