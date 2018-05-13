/*
 * BLDC_Motors.c
 *
 *  Created on: 25.04.2018
 *      Author: Norbi
 */

#include "BLDC_Motors.h"


//Range 0-100
static TIM_HandleTypeDef* BLDC_MotorsTimer;
static uint32_t leftChannel;
static uint32_t rightChannel;

static int32_t leftPower = 0;
static int32_t rightPower = 0;


void initializeBLDC_Motors(TIM_HandleTypeDef* timer, uint32_t leftCh, uint32_t rightCh) {
	BLDC_MotorsTimer = timer;
	leftChannel = leftCh;
	rightChannel = rightCh;

	HAL_TIM_PWM_Start(BLDC_MotorsTimer, leftChannel);
	HAL_TIM_PWM_Start(BLDC_MotorsTimer, rightChannel);
}


void setBLDC_MotorsDIR(uint8_t leftDir, uint8_t rightDir){

	if (leftDir==0 && rightDir==0){				//Forward
		HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET);
	} else if(leftDir==1 && rightDir==1) {		//Backward
		HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_SET);
	} else if(leftDir==1 && rightDir==0) {		//Left
		HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin|LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET);
	} else if(leftDir==0 && rightDir==1) {		//Right
		HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin|LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET);
	}

}


void setPwmDuty(TIM_HandleTypeDef* timer, uint32_t channel, uint8_t duty) {
	switch(channel){
		case TIM_CHANNEL_1:
			timer->Instance->CCR1 = duty;
			break;
		case TIM_CHANNEL_2:
			timer->Instance->CCR2 = duty;
			break;
	}
}

void setBLDC_MotorsPower(int32_t left, int32_t right) {

	setPwmDuty(BLDC_MotorsTimer, leftChannel, left);
	setPwmDuty(BLDC_MotorsTimer, rightChannel, right);
	leftPower = left;
	rightPower = right;
}

void Set_Left_Motor_Speed(int32_t left) {
	setPwmDuty(BLDC_MotorsTimer, leftChannel, left);
	leftPower = left;
}


void Set_Right_Motor_Speed(int32_t right) {
	setPwmDuty(BLDC_MotorsTimer, rightChannel, right);
	rightPower = right;
}

