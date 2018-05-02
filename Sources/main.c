#include "MPC5604B.h"
#include "pit.h"
#include "uart.h"
#include "initModeAndClock.h"
#include "interrupt.h"
#include "test.h"
#include "pid.h"
#include "control.h"
#include "electromagnet.h"
#include "delay.h"
#include "encoder.h"

extern uint8_t FLAG;
extern uint8_t data[4];
extern float pram[4];
static Motor_t Motor[5];
static ECD_t ecd[4];
extern int points[7];
//extern X_distance;
//extern Y_distance;
//extern EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
//extern EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
int main(void) 
{
//	PIT__config(PIT_Timer1,10,64,test,10);
	initLINFlex_0_UART(12);
	initModesAndClock(); 
	enableIrq();
	PIT__config(PIT_Timer1,10,64,test,10);
	motor_config(&(Motor[0]),EMIOS_CH2,EMIOS_CH11);
	motor_config(&(Motor[1]),EMIOS_CH3,EMIOS_CH12);
	motor_config(&(Motor[2]),EMIOS_CH5,EMIOS_CH13);
	motor_config(&(Motor[3]),EMIOS_CH6,EMIOS_CH14);
	Encoder__config(&ecd[0],EMIOS_CH16,0.05,260,10,0.032,17);
	Encoder__config(&ecd[1],EMIOS_CH16,0.05,260,10,0.032,17);
	Encoder__config(&ecd[2],EMIOS_CH16,0.05,260,10,0.032,17);
	Encoder__config(&ecd[3],EMIOS_CH16,0.05,260,10,0.032,17);
	Encoder__init(&ecd[0]);
	Encoder__init(&ecd[1]);
	Encoder__init(&ecd[2]);
	Encoder__init(&ecd[3]);
	while(1) {;}
}

//void move()
//{
//	int i=0;
//	
//	switch(data[3])
//	{
//	case 'X':
//		Motor[4].x_distance=(points[0]*100+points[1]*10+points[2])/100;
//		x_control_update(&(Motor));
//	case 'y':
//		Motor[4].y_distance=(points[0]*100+points[1]*10+points[2])/100;
//		y_control_update(Motor);
//	default:
//		for(i=0;i<100;i++)
//		{
//			motor_output(&(Motor[0]),0);
//			motor_output(&(Motor[1]),0);
//			motor_output(&(Motor[2]),0);
//			motor_output(&(Motor[3]),0);
//		};
//	}
//}

void test()
{
//	char temp;
	int i=0;
//	PIT__config(PIT_Timer1,10,64,test,10);
//	motor_config(Motor,forward_ch_r,backward_ch_r);
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
	for(i=0;i<100;i++)
	{motor_output(&(Motor[0]),0.5);}
	
//	Speed__bekommen(&ecd);
	PIT__clear_flag(PIT_Timer1);
}

