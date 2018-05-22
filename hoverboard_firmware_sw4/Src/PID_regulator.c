/*
 * PID_regulator.c
 *
 *  Created on: 15.05.2018
 *      Author: Norbi
 */

#include "PID_regulator.h"
#include "stm32f4xx_hal.h"

#define ERR_SUM_MAX		1000

float Kp = 40;                   // (P)roportional Tuning Parameter
float Ki = 0;					// (I)ntegral Tuning Parameter
float Kd = 3;					// (D)erivative Tuning Parameter

float last_angle;               // Keeps track of error over time
float iTerm;              		// Used to accumulate error (integral)

int16_t outMax = 1000;
int16_t outMin = -1000;

uint8_t lastOut = 16;

//float targetAngle = 0;       	// Can be adjusted according to centre of gravity


int16_t PID_calculate(float set_angle, float angle)
{

	int16_t PID_Result = 0;
    // Calculate time since last time PID was called (~10ms)
    unsigned long this_Time = HAL_GetTick();	//TODO pobrac czas z systicka
    float timeChange = this_Time - last_Time;
    lastOut = timeChange;

	// Calculate Error
	float error = set_angle - angle;

	// Calculate our PID terms
	float pTerm = Kp * error;
	iTerm += Ki * error * timeChange;
	float dTerm = Kd * (angle - last_angle) / timeChange;

	if (iTerm > ERR_SUM_MAX) {
		iTerm = ERR_SUM_MAX;
	} else if (iTerm < -ERR_SUM_MAX) {
		iTerm = -ERR_SUM_MAX;
	}

	last_angle = angle;
	last_Time = this_Time;

    // Set PWM Value
    float PID_Value = pTerm + iTerm - dTerm;

    // Limits PID to max motor speed
    if (PID_Value > 1000) PID_Value = 1000;
    else if (PID_Value < -1000) PID_Value = -1000;

/*
	   int8_t PID_Result = 0;
	   unsigned long now = HAL_GetTick();
	   unsigned long timeChange = (now - last_Time);

	  //Compute all the working error variables/
	  float input = angle;
	  float error = set_angle - input;
	  iTerm+= (Ki * error);
	  if(iTerm > outMax) iTerm= outMax;
	  else if(iTerm < outMin) iTerm= outMin;
	  double dInput = (input - last_angle);

	  //Compute PID Output/
	  double PID_Value = Kp * error + iTerm- Kd * dInput;


	  if(PID_Value > outMax) PID_Value = outMax;
	  else if(PID_Value < outMin) PID_Value = outMin;
	  PID_Result = PID_Value;

	  //Remember some variables for next time//
	  last_angle = input;
	  last_Time = now;
*/
    // Return PID Output
    PID_Result = PID_Value;
    return PID_Result;

}
