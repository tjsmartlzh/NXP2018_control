#include "MPC5604B.h"
#include "pit.h"
#include "uart.h"
#include "initModeAndClock.h"
#include "interrupt.h"
#include "test.h"
#include "control.h"
#include "electromagnet.h"
#include "delay.h"

extern uint8_t FLAG;
extern uint8_t data[4];
extern float pram[4];
extern struct motor Motor[4];
extern EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
extern EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
int main(void) 
{
	PIT__config(PIT_Timer1,10,64,test,10);
	initLINFlex_0_UART(12);
	initModesAndClock(); 
	enableIrq();
	motor_config(Motor,forward_ch_r,backward_ch_r);
//	Encoder__config(&ecd,EMIOS_CH16,0.17105,512,10,0.032,17);
//	Encoder__init(&ecd);
	while(1)  ;
}

void test()
{
//	char temp;
	float test_duty;
//	PIT__config(PIT_Timer1,10,64,test,10);
	PIT__clear_flag(PIT_Timer1);
//	motor_config(Motor,forward_ch_r,backward_ch_r);
	if(FLAG)
	{
		switch(data[3])
		{
		case 'A':
			test_duty=pram[0];
			break;
		case 'B':
			test_duty=-pram[0];
			break;
		default:
			test_duty=0;
		}
		FLAG=0;
	}
	motor_output(Motor[0],test_duty);
	open_elecm();
	delay_ms(1000);
}

