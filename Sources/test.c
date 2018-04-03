/*
 * test.c
 *
 *  Created on: Mar 27, 2018
 *      Author: dell
 */

#include "MPC5604B.h"
#include "test.h"
#include "pit.h"
//#include "uart.h"
#include "control.h"
//extern uint8_t FLAG;
//extern uint8_t data[4];
//extern float pram[4];
//extern motor_output(struct motor motor , float duty);



//extern struct motor Motor[4];
//void test()
//{
////	char temp;
//	float test_duty;
////	PIT__config(PIT_Timer1,10,64,test,10);
//	PIT__clear_flag(PIT_Timer1);
////	motor_config(Motor,forward_ch_r,backward_ch_r);
//	if(FLAG)
//	{
//		switch(data[3])
//		{
//		case 'A':
//			test_duty=pram[0];
//			break;
//		case 'B':
//			test_duty=-pram[0];
//			break;
//		default:
//			test_duty=0;
//		}
//		FLAG=0;
//	}
//	motor_output(Motor[0],test_duty);
//}
