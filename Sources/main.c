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
extern float X_location,Y_location,X_error,Y_error,theta; 
extern float Target_D_X,Target_D_Y;
static float D=0;
static float Actual_D;
extern uint8_t Elec_flag;
extern int Step_Count,Step_Count_R;
extern uint8_t Start_Flag,Send_Flag;
extern float destination[2][50];
float Target_D;
extern int step;
char str[4];
extern int sum_temp;
float omiga;
int straight_flag=0;
extern C_flag;
extern uint32_t time,last_time;
uint32_t time_0;
float delta;
float Target_D_X_R,Target_D_Y_R;

#define half_track_dis 0.21 //待测
#define half_wheel_dis 0.16 //待测
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
	GPIO__output__enable(71);  //E7  推拉机构
	GPIO__output__enable(45);  //C13 电磁铁
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
		if((Elec_flag==1)&&(step%2==0))
		{
			SIU.GPDO[45].B.PDO=1;
			delay_ms(500);
			SIU.GPDO[71].B.PDO=1;
			Elec_flag=0;
		}
		else if((Elec_flag==1)&&(step%2==1))
		{
			SIU.GPDO[71].B.PDO=0;
			delay_ms(500);
			SIU.GPDO[45].B.PDO=0;
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
//	OLED_Num(straight_flag);
	
	OLED_SetPointer(1,0);
	OLED_Str("v1 ");
	OLED_Num(motor_a[0]->actual_speed);
	
//	OLED_SetPointer(2,0);
//	OLED_Str("dst_X ");
//	OLED_Float(destination[0][step]);
//	
	OLED_SetPointer(3,0);
	OLED_Str("v2 ");
	OLED_Float(motor_a[1]->actual_speed);
	
//	OLED_SetPointer(4,0);
//	OLED_Str("v1 ");
//	OLED_Float(Target_D_X);
//	
	OLED_SetPointer(5,0);
	OLED_Str("v3 ");
	OLED_Float(motor_a[2]->actual_speed);
	
//	OLED_SetPointer(6,0);
//	OLED_Str("v3 ");
//	OLED_Float(motor_a[2]->actual_speed);
	
	OLED_SetPointer(7,0);
	OLED_Str("v4 ");
	OLED_Float(motor_a[3]->actual_speed);
	
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
//	float duty_temp_1;
	int i=0;
//	float Target_Speed;
	float temp;
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
	
	if(fabs(theta - 900)>80) omiga=-0.44;
	if((fabs(theta - 900)<=80)&&(fabs(theta - 900)>-80)) omiga=-0.55*(theta - 900)/100.0f; //正为逆时针，负为顺时针
	if(fabs(theta - 900)<=-80) omiga=0.44;
	motor_a[0]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[1]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[2]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[3]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	
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
	
	time=STM.CNT.R;
	if(time<last_time)
	{
		time_0=time+0xffffffff-last_time;
	}
	else
	{
		time_0=time-last_time;
	}
	delta=time_0/1000000.0f;
	
	if((motor_a[0]->actual_speed)*(motor_a[1]->actual_speed)>0)
	{
		Target_D_Y_R=Target_D_Y+delta*(motor_a[0]->actual_speed);
	}
	else if((motor_a[0]->actual_speed)*(motor_a[1]->actual_speed)<0)
	{
		Target_D_X_R=Target_D_X-delta*(motor_a[0]->actual_speed);
	}
	
	if((fabs(Target_D_X)<=5)||(X_location<18)||(X_location>282)) straight_flag=1; //382
	
	if((Target_D_X>5)&&(straight_flag==0)) //右
	{
		motor_a[0]->target_speed=0.4-motor_a[0]->angel_speed;
		motor_a[1]->target_speed=-0.4-motor_a[1]->angel_speed;
		motor_a[2]->target_speed=0.4+motor_a[2]->angel_speed;
		motor_a[3]->target_speed=-0.4+motor_a[3]->angel_speed;
	}
	else if((Target_D_X<=-5)&&(straight_flag==0)) //左
	{
		motor_a[0]->target_speed=-0.4-motor_a[0]->angel_speed;
		motor_a[1]->target_speed=0.4-motor_a[1]->angel_speed;
		motor_a[2]->target_speed=-0.4+motor_a[2]->angel_speed;
		motor_a[3]->target_speed=0.4+motor_a[3]->angel_speed;
	}
	else if((fabs(Target_D_X)<5)&&(straight_flag==0))
	{
		straight_flag=1;
	}
	else if((Target_D_Y>5)&&(straight_flag==1))  //后
	{
		motor_a[0]->target_speed=-0.4-motor_a[0]->angel_speed;
		motor_a[1]->target_speed=-0.4-motor_a[1]->angel_speed;
		motor_a[2]->target_speed=-0.4+motor_a[2]->angel_speed;
		motor_a[3]->target_speed=-0.4+motor_a[3]->angel_speed;
	}
	else if((Target_D_Y<=-5)&&(straight_flag==1))  //前
	{
		motor_a[0]->target_speed=0.4-motor_a[0]->angel_speed;
		motor_a[1]->target_speed=0.4-motor_a[1]->angel_speed;
		motor_a[2]->target_speed=0.4+motor_a[2]->angel_speed;
		motor_a[3]->target_speed=0.4+motor_a[3]->angel_speed;
	}
	else if(((fabs(Target_D_X)>5)&&(fabs(Target_D_Y)<=5)&&(straight_flag==1))||(Y_location<18)||(Y_location>382))
	{
		straight_flag=0;
	}
//	else if((fabs(Target_D_X)<=5)&&(fabs(Target_D_Y)<=5)&&(straight_flag==1))
//	{
//		motor_output(motor_a[0],0);
//		motor_output(motor_a[1],0);
//		motor_output(motor_a[2],0);
//		motor_output(motor_a[3],0);
//	}
	
	if((fabs(Target_D_X)<=5)&&(fabs(Target_D_Y)<=5)&&(straight_flag==1))
	{
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		Elec_flag=1;
		straight_flag=0;
		delay_ms(200);
//		step++;
		return;
	}

	PID__config(&motor_a[0]->motor_pid,0.16f,1.00f,0,10,10,70,10); //需要对PID分段
	PID__config(&motor_a[1]->motor_pid,0.16f,1.00f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.16f,1.00f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.16f,1.00f,0,10,10,70,10);
	
//	Actual_D += ecd[0]._speed*0.01;
	if(motor_a[0]->target_speed==-(motor_a[1]->target_speed))
	{
		motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[1]->actual_speed), motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), -(motor_a[3]->actual_speed), motor_a[0]->actual_speed);
		motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[0]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), -(motor_a[2]->actual_speed), motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
		motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[1]->actual_speed), motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), -(motor_a[3]->actual_speed), motor_a[2]->actual_speed);
		motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[0]->actual_speed), motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), -(motor_a[2]->actual_speed), motor_a[3]->actual_speed);
	}
	else
	{
		motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[1]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[2]->actual_speed, motor_a[0]->actual_speed)+PID__update(&(motor_a[0]->motor_pid), motor_a[3]->actual_speed, motor_a[0]->actual_speed);
		motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[0]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[2]->actual_speed, motor_a[1]->actual_speed)+PID__update(&(motor_a[1]->motor_pid), motor_a[3]->actual_speed, motor_a[1]->actual_speed);
		motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[0]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[1]->actual_speed, motor_a[2]->actual_speed)+PID__update(&(motor_a[2]->motor_pid), motor_a[3]->actual_speed, motor_a[2]->actual_speed);
		motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[0]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[1]->actual_speed, motor_a[3]->actual_speed)+PID__update(&(motor_a[3]->motor_pid), motor_a[2]->actual_speed, motor_a[3]->actual_speed);
	
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
//	char str[3];
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
	Target_D=1.0f;
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
	Actual_D += ecd[0]._speed*0.01;
	if(fabs(Actual_D)>=Target_D) 
	{
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		return;
	}
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
	motor_output(motor_a[1],-duty_temp_1);
	motor_output(motor_a[2],duty_temp_1);
	motor_output(motor_a[3],-duty_temp_1);
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
//	f2s(ecd[0]._speed,str);
//	BlueTx(str);
}
