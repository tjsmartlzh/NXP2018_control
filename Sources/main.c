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
#include "includes.h"

extern uint8_t FLAG;
extern uint8_t data[4];
extern float pram[4];
static Motor_t Motor[5];
static motor motor_a[5];
static ECD_t ecd[4];
extern int points[7];
extern uint8_t pramdata[32];
extern float X_location,Y_location; 
extern float Target_D_X,Target_D_Y;
static float D=0;
static float Actual_D;
extern uint8_t Elec_flag;
extern int Step_Count,Step_Count_R;
extern uint8_t Start_Flag,Send_Flag;
extern float destination[2][50];
float Target_D;
extern int step;
extern X_distance_L,X_distance_R,Y_distance_B,Y_distance_F; 
char str[4];
extern int sum_temp;
float omiga;

#define half_track_dis 0.25 //待测
#define half_wheel_dis 0.25 //待测
//extern X_distance;
//extern Y_distance;
//extern EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
//extern EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
void Mode0_Quick(void);
int main(void) 
{
	initModesAndClock(); 
	enableIrq();
	initLINFlex_0_UART(12);
	OLED_Init();
	
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
	GPIO__output__enable(12);
	GPIO__output__enable(20);  //B4  推拉机构
	GPIO__output__enable(21);  //B5 电磁铁
	PIT__config(PIT_Timer1,10,64,test1,10);
	str[0]=0x66;
	str[1]=0x55;
	str[2]=0x44;
	str[3]=0xff;
	while(1)
	{
//		BlueTx("000S");
//		f2s(123,str);
//		BlueTx(str);
//		Mode0_Quick();
		if((Elec_flag==1)&(step%2==0))
		{
			SIU.GPDO[21].B.PDO=1;
			delay_ms(200);
			SIU.GPDO[20].B.PDO=1;
			Elec_flag=0;
		}
		else if((Elec_flag==1)&(step%2==1))
		{
			SIU.GPDO[20].B.PDO=0;
			delay_ms(200);
			SIU.GPDO[21].B.PDO=0;
			Elec_flag=0;
		}
		else
		{
			;
		}
	}
}

void Mode0_Quick(void)
{
	OLED_Fill(0x00);
	
//	OLED_SetPointer(0,0);
//	OLED_Str("step ");
//	OLED_Num(Step_Count_R);
	
	OLED_SetPointer(1,0);
	OLED_Str("step ");
	OLED_Num(step);
	
	OLED_SetPointer(2,0);
	OLED_Str("dst_X ");
	OLED_Float(destination[0][step]);
//	
	OLED_SetPointer(3,0);
	OLED_Str("dst_Y ");
	OLED_Float(destination[1][step]);
//	
	OLED_SetPointer(4,0);
	OLED_Str("v1 ");
	OLED_Float(Target_D_X);
//	
	OLED_SetPointer(5,0);
	OLED_Str("v2 ");
	OLED_Float(Target_D_Y);
//	
	OLED_SetPointer(6,0);
	OLED_Str("v3 ");
	OLED_Float(motor_a[0]->target_speed);
	
	OLED_SetPointer(7,0);
	OLED_Str("v4 ");
	OLED_Float(motor_a[0]->actual_speed);
	
//	OLED_SetPointer(7,0);
//	OLED_Str("");
//	OLED_Float(motor_a[3]->target_speed);
//	OLED_SetPointer(6,0);
//	OLED_Str("tsr_PWM ");
//	OLED_Num(258);
//	
//	OLED_SetPointer(7,0);
//	OLED_Str("MODE ");
//	OLED_Num(5);
}

void test1()
{
	int duty_temp=0;
	float duty_temp_1;
	int i=0;
	float Target_Speed;
	Mode0_Quick();
	if((Start_Flag==0)||(step>=Step_Count)) 
	{
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		return;
	}
	SIU.GPDO[12].B.PDO=!SIU.GPDO[12].B.PDO;
	if(Send_Flag==1)
	{
		delay_ms(500);
		BlueTx(str);
		Send_Flag=0;
	}
//	if(fabs(theta - 90)>10) omiga=5;
//	if((fabs(theta - 90)<10)&&(fabs(theta - 90)>8)) omiga=4;
//	if((fabs(theta - 90)<8)&&(fabs(theta - 90)>6)) omiga=3;
//	if((fabs(theta - 90)<6)&&(fabs(theta - 90)>4)) omiga=2;
//	if((fabs(theta - 90)<4)&&(fabs(theta - 90)>2)) omiga=1;
//	if(fabs(theta - 90)<2) omiga=0;
//	if((fabs(theta - 90)<-2)&&(fabs(theta - 90)>-4)) omiga=-1;
//	if((fabs(theta - 90)<-4)&&(fabs(theta - 90)>-6)) omiga=-2;
//	if((fabs(theta - 90)<-6)&&(fabs(theta - 90)>-8)) omiga=-3;
//	if((fabs(theta - 90)<-8)&&(fabs(theta - 90)>-10)) omiga=-4;
//	if((fabs(theta - 90)<-10)) omiga=-5;
	PID__config(&motor_a[0]->motor_pid,0.58f,1.25f,0,10,10,70,10); //1.44p 50i
	PID__config(&motor_a[1]->motor_pid,0.58f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.58f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.58f,1.25f,0,10,10,70,10);
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
	//坐标系原点为左上角
	if((Target_D_X>5)&&(fabs(Target_D_Y)>5)) //右
	{
		motor_a[0]->target_speed=1.0;
		motor_a[1]->target_speed=-1.0;
		motor_a[2]->target_speed=1.0;
		motor_a[3]->target_speed=-1.0;
	}
	else if((Target_D_X<-5)&&(fabs(Target_D_Y)>5)) //左
	{
		motor_a[0]->target_speed=-1.0;
		motor_a[1]->target_speed=1.0;
		motor_a[2]->target_speed=-1.0;
		motor_a[3]->target_speed=1.0;
	}
	else if((Target_D_X>5)&&(fabs(Target_D_Y)<5)) //右
	{
		motor_a[0]->target_speed=1.0;
		motor_a[1]->target_speed=-1.0;
		motor_a[2]->target_speed=1.0;
		motor_a[3]->target_speed=-1.0;
	}
	else if((Target_D_X<-5)&&(fabs(Target_D_Y)<5)) //左
	{
		motor_a[0]->target_speed=-1.0;
		motor_a[1]->target_speed=1.0;
		motor_a[2]->target_speed=-1.0;
		motor_a[3]->target_speed=1.0;
	}
	else if((fabs(Target_D_X)<5)&&(Target_D_Y>5))  //后
	{
		motor_a[0]->target_speed=-1.0;
		motor_a[1]->target_speed=-1.0;
		motor_a[2]->target_speed=-1.0;
		motor_a[3]->target_speed=-1.0;
	}
	else if((fabs(Target_D_X)<5)&&(Target_D_Y<-5))  //前
	{
		motor_a[0]->target_speed=1.0;
		motor_a[1]->target_speed=1.0;
		motor_a[2]->target_speed=1.0;
		motor_a[3]->target_speed=1.0;
	}
	
	
	if((fabs(Target_D_X)<5)&(fabs(Target_D_Y)<5)&(step<Step_Count))
	{
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		Elec_flag=1;
		delay_ms(200);
//		step++;
		return;
	}
	
//	Actual_D += ecd[0]._speed*0.01;
	if(motor_a[0]->target_speed==-(motor_a[1]->target_speed))
	{
		motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[1]->actual_speed), motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[3]->actual_speed), motor_a[0]->actual_speed);
		motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[0]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[2]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
		motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[1]->actual_speed), motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[3]->actual_speed), motor_a[2]->actual_speed);
		motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[0]->actual_speed), motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[2]->actual_speed), motor_a[3]->actual_speed);
	}
	else
	{
		motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[1]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[3]->actual_speed, motor_a[0]->actual_speed);
		motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[0]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[2]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
		motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[1]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[3]->actual_speed, motor_a[2]->actual_speed);
		motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[0]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[2]->actual_speed, motor_a[3]->actual_speed);
	
	}
	if(motor_a[0]->duty>1.0f) motor_a[0]->duty=1.0f;
	if(motor_a[0]->duty<-1.0f) motor_a[0]->duty=-1.0f;
	if(motor_a[1]->duty>1.0f) motor_a[1]->duty=1.0f;
	if(motor_a[1]->duty<-1.0f) motor_a[1]->duty=-1.0f;
	if(motor_a[2]->duty>1.0f) motor_a[2]->duty=1.0f;
	if(motor_a[2]->duty<-1.0f) motor_a[2]->duty=-1.0f;
	if(motor_a[3]->duty>1.0f) motor_a[3]->duty=1.0f;
	if(motor_a[3]->duty<-1.0f) motor_a[3]->duty=-1.0f;
//	duty_temp_1=0.4;
	motor_output(motor_a[0],motor_a[0]->duty);
	motor_output(motor_a[1],motor_a[1]->duty);
	motor_output(motor_a[2],motor_a[2]->duty);
	motor_output(motor_a[3],motor_a[3]->duty);
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
//	f2s(ecd[0]._speed,str);
//	BlueTx(str);
	PIT__clear_flag(PIT_Timer1);
}

void test()
{
	int duty_temp=0;
	float duty_temp_1;
	int i=0;
	char str[3];
	PID__config(&motor_a[0]->motor_pid,0.64f,1.25f,0,10,10,70,10); //1.44p 50i
	PID__config(&motor_a[1]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.64f,1.25f,0,10,10,70,10);
//	switch(data[3])
//	{
//	case 'x':
//	{
//		motor_a[0]->target_speed=-1.0;
//		motor_a[1]->target_speed=1.0;
//		motor_a[2]->target_speed=-1.0;
//		motor_a[3]->target_speed=1.0;
//		Target_D=X_distance_R;
//		break;
//	}
//	case 'X':
//	{
//		motor_a[0]->target_speed=1.0;
//		motor_a[1]->target_speed=-1.0;
//		motor_a[2]->target_speed=1.0;
//		motor_a[3]->target_speed=-1.0;
//		Target_D=X_distance_L;
//		break;
//	}
//	case 'y':
//	{
		motor_a[0]->target_speed=1.0;
		motor_a[1]->target_speed=1.0;
		motor_a[2]->target_speed=1.0;
		motor_a[3]->target_speed=1.0;
//		Target_D=Y_distance_B;
//		break;
//	}
//	case 'Y':
//	{
//		motor_a[0]->target_speed=-1.0;
//		motor_a[1]->target_speed=-1.0;
//		motor_a[2]->target_speed=-1.0;
//		motor_a[3]->target_speed=-1.0;
//		Target_D=Y_distance_F;
//		break;
//	}
//	}
	Target_D=3.0f;
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
//	switch(data[3])
//	{
//	case 'x':
//	{
//	
//	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[1]->actual_speed), motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[3]->actual_speed), motor_a[0]->actual_speed);
//	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[0]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[2]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
//	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[1]->actual_speed), motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[3]->actual_speed), motor_a[2]->actual_speed);
//	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[0]->actual_speed), motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[2]->actual_speed), motor_a[3]->actual_speed);
////	break;
//	}
//	case 'X':
//	{
//	
//	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[1]->actual_speed), motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[3]->actual_speed), motor_a[0]->actual_speed);
//	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[0]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[2]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
//	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[1]->actual_speed), motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[3]->actual_speed), motor_a[2]->actual_speed);
//	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[0]->actual_speed), motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[2]->actual_speed), motor_a[3]->actual_speed);
//	break;
//	}
//	case 'y':
//	{
	//motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed);//+PID__update(&(motor_a[0]->motor_pid), motor_a[1]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[3]->actual_speed, motor_a[0]->actual_speed);
	//motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed);//+PID__update(&(motor_a[1]->motor_pid), motor_a[0]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[2]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
	//motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed);//+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[1]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[3]->actual_speed, motor_a[2]->actual_speed);
	//motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed);//+PID__update(&(motor_a[3]->motor_pid), motor_a[0]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[2]->actual_speed, motor_a[3]->actual_speed);
//	}
//	case 'Y':
//	{
//	motor_a[0]->duty=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[1]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[3]->actual_speed, motor_a[0]->actual_speed);
//	motor_a[1]->duty=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[0]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[2]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
//	motor_a[2]->duty=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[1]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[3]->actual_speed, motor_a[2]->actual_speed);
//	motor_a[3]->duty=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[0]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[2]->actual_speed, motor_a[3]->actual_speed);
//	break;
//	}
//	}
	//Actual_D += ecd[0]._speed*0.01;
	//if(fabs(Actual_D)>=Target_D) 
	//{
		//motor_output(motor_a[0],0);
		//motor_output(motor_a[1],0);
		//motor_output(motor_a[2],0);
		//motor_output(motor_a[3],0);
		//return;
	//}
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
	motor_output(motor_a[0],duty_temp_1);
	motor_output(motor_a[1],duty_temp_1);
	motor_output(motor_a[2],duty_temp_1);
	motor_output(motor_a[3],duty_temp_1);
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
//	f2s(ecd[0]._speed,str);
//	BlueTx(str);
}
