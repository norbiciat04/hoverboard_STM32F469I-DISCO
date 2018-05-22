/*
 * BLDC_Motors.h
 *
 *  Created on: 25.04.2018
 *      Author: Norbi
 */

#ifndef BLDC_MOTORS_H_
#define BLDC_MOTORS_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"

//????
//static const int32_t LED_POWER_RETAIN = -1;


#define L_MOT_FORWARD	HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET)
#define L_MOT_BACKWARD	HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET)

#define R_MOT_FORWARD	HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET)
#define R_MOT_BACKWARD	HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_SET)

#define FORWARD		1
#define BACKWARD	0

#define LEFT_MOTOR	1
#define RIGHT_MOTOR	2
#define BOTH_MOTORS	3

void setPwmDuty(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t duty);

void Initialize_LR_Motors(TIM_HandleTypeDef* timer, uint32_t leftCh, uint32_t rightCh);

void Set_LR_Motors_DIR(uint8_t leftDir, uint8_t rightDir);
void Set_LR_Motors_Speed(uint16_t left, uint16_t right);
void Stop_LR_Motors(void);

void Set_Left_Motor_Speed(uint16_t left);
void Set_Left_Motor_DIR(uint8_t dir);

void Set_Right_Motor_Speed(uint16_t right);
void Set_Right_Motor_DIR(uint8_t dir);

void Control_Motor_by_PID(int8_t motor, int16_t pid_value);

/*
int32_t getLedRedPower(void);
int32_t getLedGreenPower(void);
TODO - get rpm of motors
*/
#endif /* BLDC_MOTORS_H_ */
