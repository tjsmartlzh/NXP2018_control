#include "MPC5604B.h"
#include "pit.h"
#include "uart.h"
#include "initModeAndClock.h"
#include "interrupt.h"
#include "test.h"
#include "control.h"
#include "electromagnet.h"
#include "delay.h"
#include "encoder.h"

extern uint8_t FLAG;
extern uint8_t data[4];
extern float pram[4];
static Motor_t Motor[4];
static ECD_t ecd;
float speed;
//extern EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
//extern EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
int main(void) 
{
//	PIT__config(PIT_Timer1,10,64,test,10);
//	initLINFlex_0_UART(12);
	initModesAndClock(); 
	enableIrq();
	PIT__config(PIT_Timer1,10,64,test,10);
	motor_config(&(Motor[0]),EMIOS_CH2,EMIOS_CH11);
	motor_config(&(Motor[1]),EMIOS_CH3,EMIOS_CH12);
	motor_config(&(Motor[2]),EMIOS_CH5,EMIOS_CH13);
	motor_config(&(Motor[3]),EMIOS_CH6,EMIOS_CH14);
	Encoder__config(&ecd,EMIOS_CH16,0.17105,512,10,0.032,17);
	Encoder__init(&ecd);
	while(1) {;}
}

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
//	motor_output(&(Motor[0]),0.5);
//	motor_output(&(Motor[0]),test_duty);
	for(i=0;i<100;i++)
	{motor_output(&(Motor[0]),0.5);}
	speed = Speed__bekommen(&ecd);
//	pwm__duty_update(&(Motor[0])->forward_ch,0.5);
	PIT__clear_flag(PIT_Timer1);
//	delay_ms(1000);
}

