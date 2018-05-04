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
static motor motor_a[5];
static ECD_t ecd[4];
extern float points[7];
//extern X_distance;
//extern Y_distance;
//extern EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
//extern EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
int main(void) 
{
//	PIT__config(PIT_Timer1,10,64,test,10);
	initModesAndClock(); 
	enableIrq();
	motor_a[0]=&(Motor[0]);
	motor_a[1]=&(Motor[1]);
	motor_a[2]=&(Motor[2]);
	motor_a[3]=&(Motor[3]);
	motor_a[4]=&(Motor[4]);
	motor_config(&(Motor[0]),EMIOS_CH3,EMIOS_CH9,0,0,0,10,10,10,10);
	motor_config(&(Motor[1]),EMIOS_CH5,EMIOS_CH10,0,0,0,10,10,10,10);
	motor_config(&(Motor[2]),EMIOS_CH6,EMIOS_CH11,0,0,0,10,10,10,10);
	motor_config(&(Motor[3]),EMIOS_CH7,EMIOS_CH13,0,0,0,10,10,10,10);
	PIT__config(PIT_Timer1,10,64,test,10);
//	motor_config(Motor[1],EMIOS_CH3,EMIOS_CH12);
//	motor_config(Motor[2],EMIOS_CH5,EMIOS_CH13);
//	motor_config(Motor[3],EMIOS_CH6,EMIOS_CH14);
	Encoder__config(&ecd[0],EMIOS_CH16,0.05,260,10,0.032,17);
	Encoder__config(&ecd[1],EMIOS_CH17,0.05,260,10,0.032,18);
	Encoder__config(&ecd[2],EMIOS_CH18,0.05,260,10,0.032,19);
	Encoder__config(&ecd[3],EMIOS_CH19,0.05,260,10,0.032,20);
	Encoder__init(&ecd[0]);
	Encoder__init(&ecd[1]);
	Encoder__init(&ecd[2]);
	Encoder__init(&ecd[3]);
	while(1){;}
}

//void move()
//{
//	int i=0;
//	
//	switch(data[3])
//	{
//	case 'X':
//		Motor[4].x_distance=(points[0]*100+points[1]*10+points[2])/100;
//		x_control_update(motor_a);
//	case 'y':
//		Motor[4].y_distance=(points[0]*100+points[1]*10+points[2])/100;
//		y_control_update(motor_a);
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
	float duty[4];
	int i=0;
	PIT__clear_flag(PIT_Timer1);
	motor_a[0]->target_speed=1.0;
	motor_a[1]->target_speed=1.0;
	motor_a[2]->target_speed=1.0;
	motor_a[3]->target_speed=1.0;
	Speed__bekommen(&ecd[0]);
	Speed__bekommen(&ecd[1]);
	Speed__bekommen(&ecd[2]);
	Speed__bekommen(&ecd[3]);
	duty[0]=duty[0]+PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, ecd[0]._speed);
	duty[1]=duty[1]+PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, ecd[1]._speed);
	duty[2]=duty[2]+PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, ecd[2]._speed);
	duty[3]=duty[3]+PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, ecd[3]._speed);
	if(Dir__bekommen(&ecd[0])) 
	ecd[0]._speed = -ecd[0]._speed;
	motor_output(&(Motor[0]),-0.8);
	motor_output(&(Motor[1]),-0.8);
	motor_output(&(Motor[2]),-0.8);
	motor_output(&(Motor[3]),-0.8);
	
}

