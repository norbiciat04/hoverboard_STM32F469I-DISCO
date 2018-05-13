/*
 * BLDC_Motors.h
 *
 *  Created on: 25.04.2018
 *      Author: Norbi
 */

#ifndef BLDC_MOTORS_H_
#define BLDC_MOTORS_H_

#include "stm32f4xx_hal.h"

//????
//static const int32_t LED_POWER_RETAIN = -1;


#define L_MOT_FORWARD	HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET)
#define L_MOT_BACKWARD	HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET)

#define R_MOT_FORWARD	HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET)
#define R_MOT_BACKWARD	HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_SET)

void setPwmDuty(TIM_HandleTypeDef* timer, uint32_t channel, uint8_t duty);
void initializeBLDC_Motors(TIM_HandleTypeDef* timer, uint32_t leftCh, uint32_t rightCh);
void setBLDC_MotorsPower(int32_t left, int32_t right);
void shutdownBLDC_Motors(void);
void setBLDC_MotorsDIR(uint8_t leftDir, uint8_t rightDir);

void Set_Left_Motor_Speed(int32_t left);
void Set_Right_Motor_Speed(int32_t right);

/*
int32_t getLedRedPower(void);
int32_t getLedGreenPower(void);
TODO - get rpm of motors
*/
#endif /* BLDC_MOTORS_H_ */
