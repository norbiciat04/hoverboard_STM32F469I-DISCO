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

static uint8_t leftPower = 0;
static uint8_t rightPower = 0;


void Initialize_LR_Motors(TIM_HandleTypeDef* timer, uint32_t leftCh, uint32_t rightCh) {
	BLDC_MotorsTimer = timer;
	leftChannel = leftCh;
	rightChannel = rightCh;

	HAL_TIM_PWM_Start(BLDC_MotorsTimer, leftChannel);
	HAL_TIM_PWM_Start(BLDC_MotorsTimer, rightChannel);
}


void Set_LR_Motors_DIR(uint8_t leftDir, uint8_t rightDir){

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

void Set_LR_Motors_Speed(uint8_t left, uint8_t right) {

	setPwmDuty(BLDC_MotorsTimer, leftChannel, left);
	setPwmDuty(BLDC_MotorsTimer, rightChannel, right);
	leftPower = left;
	rightPower = right;
}

void Stop_LR_Motors(void) {

	setPwmDuty(BLDC_MotorsTimer, leftChannel, 0);
	setPwmDuty(BLDC_MotorsTimer, rightChannel, 0);
}

void Set_Left_Motor_Speed(uint8_t left) {
	setPwmDuty(BLDC_MotorsTimer, leftChannel, left);
	leftPower = left;
}

void Set_Right_Motor_Speed(uint8_t right) {
	setPwmDuty(BLDC_MotorsTimer, rightChannel, right);
	rightPower = right;
}

void Set_Left_Motor_DIR(uint8_t dir)
{
	switch (dir) {
	   case FORWARD:
			HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET); //Forward
	     break;
	   case BACKWARD:
			HAL_GPIO_WritePin(GPIOC, LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET); //Backward
	     break;

	   default:
	     break;
	 }
}

void Set_Right_Motor_DIR(uint8_t dir)
{
	switch (dir) {
	   case FORWARD:
			HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET); //Forward
	     break;
	   case BACKWARD:
			HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_SET); //Backward
	     break;

	   default:
	     break;
	 }
}

void Control_Motor_by_PID(int8_t motor, int8_t pid_value) {
	int8_t pid_value_s = pid_value;
	switch(motor){
		case LEFT_MOTOR:
			if(pid_value_s > 0){
				Set_Left_Motor_DIR(FORWARD);
			} else if (pid_value_s < 0) {
				Set_Left_Motor_DIR(BACKWARD);
				pid_value_s = -pid_value_s;
			}
			setPwmDuty(BLDC_MotorsTimer, leftChannel, pid_value_s);
			leftPower = pid_value_s;
			break;

		case RIGHT_MOTOR:
			if(pid_value_s > 0){
				Set_Right_Motor_DIR(FORWARD);
			} else if (pid_value_s < 0) {
				Set_Right_Motor_DIR(BACKWARD);
				pid_value_s = -pid_value_s;
			}
			setPwmDuty(BLDC_MotorsTimer, rightChannel, pid_value_s);
			rightPower = pid_value_s;
			break;
		case BOTH_MOTORS:
			if(pid_value_s > 0){
				Set_Left_Motor_DIR(FORWARD);
				Set_Right_Motor_DIR(FORWARD);
			} else if (pid_value_s < 0) {
				Set_Left_Motor_DIR(BACKWARD);
				Set_Right_Motor_DIR(BACKWARD);
				pid_value_s = -pid_value_s;
			}
			if(pid_value_s <20)
				pid_value_s = 20;
			setPwmDuty(BLDC_MotorsTimer, leftChannel, pid_value_s);
			setPwmDuty(BLDC_MotorsTimer, rightChannel, pid_value_s);
			leftPower = pid_value_s;
			rightPower = pid_value_s;
			break;

	}

}
