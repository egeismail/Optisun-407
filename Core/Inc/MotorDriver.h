/*
 * MotorDriver.h
 *
 *  Created on: Jun 27, 2022
 */
#include "main.h"
#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_


typedef unsigned char MotorDef;
#define MOTOR_0 (MotorDef)0
#define MOTOR_1 (MotorDef)1
#define MOTOR_2 (MotorDef)2
#define MOTOR_3 (MotorDef)3
#define MOTOR_4 (MotorDef)4
#define MOTOR_5 (MotorDef)5
#define PWM1CCR TIM3->CCR1




/*
 * PWM1 TIM4_CH1
PWM2 TIM4_CH2
PWM3 TIM4_CH3
PWM4 TIM4_CH4
PWM5 TIM3_CH1
PWM6 TIM3_CH2
 * */
void setSpeedFloat(MotorDef motor,float dutycycle){
	float realDelta = dutycycle>0.0 ? dutycycle : -dutycycle;
	PWM1CCR = (int)((realDelta*(float)8000));
	switch (motor) {
		case MOTOR_0:
			HAL_GPIO_WritePin(M1_A_GPIO_Port, M1_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
			HAL_GPIO_WritePin(M1_B_GPIO_Port, M1_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
			break;
		case MOTOR_1:
			HAL_GPIO_WritePin(M2_A_GPIO_Port, M2_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
			HAL_GPIO_WritePin(M2_B_GPIO_Port, M2_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
			break;
		case MOTOR_2:
			HAL_GPIO_WritePin(M3_A_GPIO_Port, M3_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
			HAL_GPIO_WritePin(M3_B_GPIO_Port, M3_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
			break;
		case MOTOR_3:
			HAL_GPIO_WritePin(M4_A_GPIO_Port, M4_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
			HAL_GPIO_WritePin(M4_B_GPIO_Port, M4_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
			break;
		case MOTOR_4:
			HAL_GPIO_WritePin(M5_A_GPIO_Port, M5_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
			HAL_GPIO_WritePin(M5_B_GPIO_Port, M5_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
			break;
		case MOTOR_5:
			HAL_GPIO_WritePin(M6_A_GPIO_Port, M6_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
			HAL_GPIO_WritePin(M6_B_GPIO_Port, M6_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
			break;
	}
}
void setSpeedInt(MotorDef motor,int dutycycle){
	int realDelta = dutycycle>0 ? dutycycle : -dutycycle;
	PWM1CCR = realDelta;
	switch (motor) {
			case MOTOR_0:
				HAL_GPIO_WritePin(M1_A_GPIO_Port, M1_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
				HAL_GPIO_WritePin(M1_B_GPIO_Port, M1_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
				break;
			case MOTOR_1:
				HAL_GPIO_WritePin(M2_A_GPIO_Port, M2_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
				HAL_GPIO_WritePin(M2_B_GPIO_Port, M2_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
				break;
			case MOTOR_2:
				HAL_GPIO_WritePin(M3_A_GPIO_Port, M3_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
				HAL_GPIO_WritePin(M3_B_GPIO_Port, M3_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
				break;
			case MOTOR_3:
				HAL_GPIO_WritePin(M4_A_GPIO_Port, M4_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
				HAL_GPIO_WritePin(M4_B_GPIO_Port, M4_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
				break;
			case MOTOR_4:
				HAL_GPIO_WritePin(M5_A_GPIO_Port, M5_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
				HAL_GPIO_WritePin(M5_B_GPIO_Port, M5_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
				break;
			case MOTOR_5:
				HAL_GPIO_WritePin(M6_A_GPIO_Port, M6_A_Pin, PWM1CCR > 0 ? (dutycycle > 0 ? SET : RESET) : RESET);
				HAL_GPIO_WritePin(M6_B_GPIO_Port, M6_B_Pin, PWM1CCR > 0 ? (dutycycle < 0 ? SET : RESET) : RESET);
				break;
	}
}
int phase0=0,phase1=150,phase2=300,phase3=450,phase4=600,phase5=750;
void testDrivingTick(int tick){
	phase0= (phase0+tick)%10000;
	phase1= (phase1+tick)%10000;
	phase2= (phase2+tick)%10000;
	phase3= (phase3+tick)%10000;
	phase4= (phase4+tick)%10000;
	phase5= (phase5+tick)%10000;
	setSpeedInt(MOTOR_0, phase0-5000);
	setSpeedInt(MOTOR_1, phase1-5000);
	setSpeedInt(MOTOR_2, phase2-5000);
	setSpeedInt(MOTOR_3, phase3-5000);
	setSpeedInt(MOTOR_4, phase4-5000);
	setSpeedInt(MOTOR_5, phase5-5000);
}
void testDriving2(){

	setSpeedInt(MOTOR_0, 2500);
	setSpeedInt(MOTOR_1, -2500);
	setSpeedInt(MOTOR_2, 2500);
	setSpeedInt(MOTOR_3, -2500);
	setSpeedInt(MOTOR_4, 2500);
	setSpeedInt(MOTOR_5, -2500);
	HAL_Delay(1000);
	setSpeedInt(MOTOR_0, -2500);
	setSpeedInt(MOTOR_1, 2500);
	setSpeedInt(MOTOR_2, -2500);
	setSpeedInt(MOTOR_3, 2500);
	setSpeedInt(MOTOR_4, -2500);
	setSpeedInt(MOTOR_5, 2500);
	HAL_Delay(1000);
}


#endif /* INC_MOTORDRIVER_H_ */
