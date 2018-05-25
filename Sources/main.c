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
#include "gpio.h"
#include "math.h"

extern uint8_t FLAG;
extern uint8_t data[4];
extern float pram[4];
static Motor_t Motor[5];
static motor motor_a[5];
static ECD_t ecd[4];
extern int points[7];
extern uint8_t pramdata[32];
extern float X_distance_L,X_distance_R,Y_distance_F,Y_distance_B; 
static float Target_D,Actual_D;
static float D=0;
//extern X_distance;
//extern Y_distance;
//extern EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
//extern EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
int main(void) 
{
//	PIT__config(PIT_Timer1,10,64,test,10);
	initModesAndClock(); 
	enableIrq();
	initLINFlex_0_UART(12);
	motor_config(&(Motor[0]),EMIOS_CH3,EMIOS_CH4,0.2,5,0,10,10,10,10);  //B11,B12
	motor_config(&(Motor[1]),EMIOS_CH5,EMIOS_CH6,0.2,0,0,10,10,10,10);  //B13,B14
	motor_config(&(Motor[2]),EMIOS_CH18,EMIOS_CH20,0.2,0,0,10,10,10,10);  //E2,E4
	motor_config(&(Motor[3]),EMIOS_CH21,EMIOS_CH22,0.2,0,0,10,10,10,10);  //E5,E6
	motor_a[0]=&(Motor[0]);
	motor_a[1]=&(Motor[1]);
	motor_a[2]=&(Motor[2]);
	motor_a[3]=&(Motor[3]);
	Encoder__config(&ecd[0],EMIOS_CH8,1,390,10,0.0574,2);  //A8,A2
	Encoder__config(&ecd[1],EMIOS_CH16,1,390,10,0.0574,47);  //E0,C15
	Encoder__config(&ecd[2],EMIOS_CH24,1,390,10,0.0574,1);  //D12,A1
	Encoder__config(&ecd[3],EMIOS_CH0,1,390,10,0.0574,41); //A0,C9原本是舵机的时基，此处暂不考虑舵机的使用，故暂用来做输入检测
	Encoder__init(&ecd[0]);
	Encoder__init(&ecd[1]); 
	Encoder__init(&ecd[2]);
	Encoder__init(&ecd[3]);
	GPIO__output__enable(14);
	GPIO__output__enable(17);
	SIU.GPDO[17].B.PDO=1;
	PIT__config(PIT_Timer1,10,64,test1,10);
//	motor_config(Motor[1],EMIOS_CH3,EMIOS_CH12);
//	motor_config(Motor[2],EMIOS_CH5,EMIOS_CH13);
//	motor_config(Motor[3],EMIOS_CH6,EMIOS_CH14);
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

void test1()
{
//	char temp;
	int duty_temp=0;
	float duty_temp_1;
	int i=0;
	char str[3];
	PID__config(&motor_a[0]->motor_pid,0.64f,1.25f,0,10,10,70,10); //1.44p 50i
	PID__config(&motor_a[1]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	switch(data[3])
	{
	case 'x':
	{
		motor_a[0]->target_speed=-1.0;
		motor_a[1]->target_speed=1.0;
		motor_a[2]->target_speed=-1.0;
		motor_a[3]->target_speed=1.0;
		Target_D=X_distance_R;
		break;
	}
	case 'X':
	{
		motor_a[0]->target_speed=1.0;
		motor_a[1]->target_speed=-1.0;
		motor_a[2]->target_speed=1.0;
		motor_a[3]->target_speed=-1.0;
		Target_D=X_distance_L;
		break;
	}
	case 'y':
	{
		motor_a[0]->target_speed=1.0;
		motor_a[1]->target_speed=1.0;
		motor_a[2]->target_speed=1.0;
		motor_a[3]->target_speed=1.0;
		Target_D=Y_distance_B;
		break;
	}
	case 'Y':
	{
		motor_a[0]->target_speed=-1.0;
		motor_a[1]->target_speed=-1.0;
		motor_a[2]->target_speed=-1.0;
		motor_a[3]->target_speed=-1.0;
		Target_D=Y_distance_F;
		break;
	}
	}
	Speed__bekommen(&ecd[0]);
	if(Dir__bekommen(&ecd[0])) ecd[0]._speed=-(ecd[0]._speed);
	Speed__bekommen(&ecd[1]);
	if(Dir__bekommen(&ecd[1])) ecd[1]._speed=-(ecd[1]._speed);
	Speed__bekommen(&ecd[2]);
	if(Dir__bekommen(&ecd[2])) ecd[2]._speed=-(ecd[2]._speed);
	Speed__bekommen(&ecd[3]);
	if(Dir__bekommen(&ecd[3])) ecd[3]._speed=-(ecd[3]._speed);
	motor_a[0]->actual_speed=ecd[0]._speed;
	motor_a[1]->actual_speed=ecd[1]._speed;
	motor_a[2]->actual_speed=ecd[2]._speed;
	motor_a[3]->actual_speed=ecd[3]._speed;
//	Actual_Speed=(ecd[0]._speed+ecd[1]._speed+ecd[2]._speed+ecd[3]._speed)/4;
	switch(data[3])
	{
	case 'x':
	{
	
	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[1]->actual_speed), motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[3]->actual_speed), motor_a[0]->actual_speed);
	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[0]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[2]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[1]->actual_speed), motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[3]->actual_speed), motor_a[2]->actual_speed);
	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[0]->actual_speed), motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[2]->actual_speed), motor_a[3]->actual_speed);
	break;
	}
	case 'X':
	{
	
	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[1]->actual_speed), motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[3]->actual_speed), motor_a[0]->actual_speed);
	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[0]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[2]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[1]->actual_speed), motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[3]->actual_speed), motor_a[2]->actual_speed);
	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[0]->actual_speed), motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[2]->actual_speed), motor_a[3]->actual_speed);
	break;
	}
	case 'y':
	{
	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[1]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[3]->actual_speed, motor_a[0]->actual_speed);
	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[0]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[2]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[1]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[3]->actual_speed, motor_a[2]->actual_speed);
	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[0]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[2]->actual_speed, motor_a[3]->actual_speed);
	break;
	}
	case 'Y':
	{
	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[1]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[3]->actual_speed, motor_a[0]->actual_speed);
	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[0]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[2]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[1]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[3]->actual_speed, motor_a[2]->actual_speed);
	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[0]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[2]->actual_speed, motor_a[3]->actual_speed);
	break;
	}
	}
	Actual_D += ecd[0]._speed*0.01;
//	f2s(-ecd[0]._speed,str);
//	BlueTx(str);
	if(fabs(Actual_D)>=Target_D) 
	{
//		motor_a[0]->target_speed=0;
//		motor_a[1]->target_speed=0;
//		motor_a[2]->target_speed=0;
//		motor_a[3]->target_speed=0;
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		return;
	}
//	duty[0]=0.1*(motor_a[0]->target_speed-ecd[0]._speed);
//	duty[1]=0.1*(motor_a[1]->target_speed-ecd[1]._speed);
//	duty[2]=0.1*(motor_a[2]->target_speed-ecd[2]._speed);
//	duty[3]=0.1*(motor_a[3]->target_speed-ecd[3]._speed);
	if(motor_a[0]->duty>1.0f) motor_a[0]->duty=1.0f;
	if(motor_a[0]->duty<-1.0f) motor_a[0]->duty=-1.0f;
	if(motor_a[1]->duty>1.0f) motor_a[1]->duty=1.0f;
	if(motor_a[1]->duty<-1.0f) motor_a[1]->duty=-1.0f;
	if(motor_a[2]->duty>1.0f) motor_a[2]->duty=1.0f;
	if(motor_a[2]->duty<-1.0f) motor_a[2]->duty=-1.0f;
	if(motor_a[3]->duty>1.0f) motor_a[3]->duty=1.0f;
	if(motor_a[3]->duty<-1.0f) motor_a[3]->duty=-1.0f;
	PIT__clear_flag(PIT_Timer1);
	duty_temp_1=0.4;
	motor_output(motor_a[0],motor_a[0]->duty);
	motor_output(motor_a[1],motor_a[1]->duty);
	motor_output(motor_a[2],motor_a[2]->duty);
	motor_output(motor_a[3],motor_a[3]->duty);
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
//	f2s(ecd[0]._speed,str);
//	BlueTx(str);
}

//void test()
//{
////	char temp;
//	float duty[4];
//	int duty_temp=0;
//	float duty_temp_1;
//	int i=0;
//	char str[3];
//	switch(data[3]) 
//	{
//	case('x'):  //左走
//		{
//			motor_a[0]->target_speed=-Target_Speed;
//			motor_a[1]->target_speed=Target_Speed;
//			motor_a[2]->target_speed=-Target_Speed;
//			motor_a[3]->target_speed=Target_Speed;
//		}
//	case('X'): //右走
//		{
//		motor_a[0]->target_speed=Target_Speed;
//		motor_a[1]->target_speed=-Target_Speed;
//		motor_a[2]->target_speed=Target_Speed;
//		motor_a[3]->target_speed=-Target_Speed;
//		}
//	case('y'):  //前走
//	{
//		motor_a[0]->target_speed=Target_Speed;
//		motor_a[1]->target_speed=Target_Speed;
//		motor_a[2]->target_speed=Target_Speed;
//		motor_a[3]->target_speed=Target_Speed;
//	}
//	case('Y'):  //后走
//	{
//		motor_a[0]->target_speed=-Target_Speed;
//		motor_a[1]->target_speed=-Target_Speed;
//		motor_a[2]->target_speed=-Target_Speed;
//		motor_a[3]->target_speed=-Target_Speed;
//	}
//	default:
//	{
//		motor_a[0]->target_speed=0;
//		motor_a[1]->target_speed=0;
//		motor_a[2]->target_speed=0;
//		motor_a[3]->target_speed=0;
//	}
//	}
//	Speed__bekommen(&ecd[0]);
//	if(Dir__bekommen(&ecd[0])) ecd[0]._speed=-(ecd[0]._speed);
//	Speed__bekommen(&ecd[1]);
//	if(Dir__bekommen(&ecd[1])) ecd[1]._speed=-(ecd[1]._speed);
//	Speed__bekommen(&ecd[2]);
//	if(Dir__bekommen(&ecd[2])) ecd[2]._speed=-(ecd[2]._speed);
//	Speed__bekommen(&ecd[3]);
//	if(Dir__bekommen(&ecd[3])) ecd[3]._speed=-(ecd[3]._speed);
//	duty[0]=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, ecd[0]._speed);
//	duty[1]=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, ecd[1]._speed);
//	duty[2]=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, ecd[2]._speed);
//	duty[3]=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, ecd[3]._speed);
//	if(duty[0]>1.0f) duty[0]=1.0f;
//	if(duty[0]<-1.0f) duty[0]=-1.0f;
//	if(duty[1]>1.0f) duty[0]=1.0f;
//	if(duty[1]<-1.0f) duty[0]=-1.0f;
//	if(duty[2]>1.0f) duty[0]=1.0f;
//	if(duty[2]<-1.0f) duty[0]=-1.0f;
//	if(duty[3]>1.0f) duty[0]=1.0f;
//	if(duty[3]<-1.0f) duty[0]=-1.0f;
//	PIT__clear_flag(PIT_Timer1);
//	motor_output(&(Motor[0]),duty[0]);
//	motor_output(&(Motor[1]),duty[1]);
//	motor_output(&(Motor[2]),duty[2]);
//	motor_output(&(Motor[3]),duty[3]);
//	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
////	f2s(123,str);
////	BlueTx(str);
//}
