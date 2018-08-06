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
#include "stm.h"
#include "disableWatchdog.h"
#include "ccd_capture.h"

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
extern float destination[2][10];
float Target_D;
extern int step;
char str[4];
extern int sum_X,sum_Y;
float omiga;
int straight_flag=0;
extern C_flag;
extern uint32_t time,last_time;
static uint32_t start_time,stop_time;
uint32_t time_0;
float delta;
float Target_D_X_R=10,Target_D_Y_R=10;
float X_location_R,Y_location_R;
extern float optimumCovariance_X, optimumCovariance_Y,kalmanGain_X, kalmanGain_Y, optimum_X, optimum_Y, estimate_X, estimate_Y;
extern float estimateCovariance_X,estimateCovariance_Y;
extern int stop_flag;
extern int die_flag;
extern int elec_flag;
extern int direction;
extern int exit_flag;
extern int mode;
static float rotating_duty;
static int finish_flag=0;
extern int enter_direction,temp;
static int j=0,i=0,g=0,k=0;
static float quick_speed,slow_speed;
extern int img[128];
extern int run_flag;
extern int times;

#define half_track_dis 0.21 
#define half_wheel_dis 0.16 
#define far_threshold 20
#define near_threshold 7

void Mode0_Quick(void);
int main(void) 
{
	initModesAndClock(); 
	enableIrq();
	disableWatchdog();
	initLINFlex_0_UART(11);
	OLED_Init();
	STM_init();
//	ccd_init();
	
	motor_config(&(Motor[0]),EMIOS_CH3,EMIOS_CH4,0,0,0,10,10,10,10);  //B11,B12
	motor_config(&(Motor[1]),EMIOS_CH5,EMIOS_CH6,0,0,0,10,10,10,10);  //B13,B14
	motor_config(&(Motor[2]),EMIOS_CH18,EMIOS_CH20,0,0,0,10,10,10,10);  //E2,E4
	motor_config(&(Motor[3]),EMIOS_CH21,EMIOS_CH22,0,0,0,10,10,10,10);  //E5,E6
	motor_a[0]=&(Motor[0]);
	motor_a[1]=&(Motor[1]);
	motor_a[2]=&(Motor[2]);
	motor_a[3]=&(Motor[3]);
	Encoder__config(&ecd[0],EMIOS_CH8,1,390,10,0.0574,48);  //A8,D0(A2)
	Encoder__config(&ecd[1],EMIOS_CH24,1,390,10,0.0574,52);  //D12,D4(A1)
	Encoder__config(&ecd[2],EMIOS_CH16,1,390,10,0.0574,47);  //E0,C15
	Encoder__config(&ecd[3],EMIOS_CH0,1,390,10,0.0574,41);  //A0,C9鍘熸湰鏄埖鏈虹殑鏃跺熀锛屾澶勬殏涓嶈�铏戣埖鏈虹殑浣跨敤锛屾晠鏆傜敤鏉ュ仛杈撳叆妫�祴
	Encoder__init(&ecd[0]);
	Encoder__init(&ecd[1]); 
	Encoder__init(&ecd[2]);
	Encoder__init(&ecd[3]);
	GPIO__output__enable(14);
	GPIO__output__enable(15);
	GPIO__output__enable(12);
	GPIO__output__enable(71);  //E7  鎺ㄦ媺鏈烘瀯
	GPIO__output__enable(45);  //C13 鐢电閾�
	GPIO__output__enable(44);  //C12鍙冲墠鏂圭數纾侀搧
	GPIO__output__enable(40);  //C8宸﹀墠鏂圭數纾侀搧
	SIU.GPDO[45].B.PDO=0; //0
	SIU.GPDO[71].B.PDO=1;
	PIT__config(PIT_Timer1,10,64,test2,10);
	str[0]=0x66;
//	str[1]=0x55;
//	str[2]=0x44;
	str[3]=0xff;
//	start_time=STM.CNT.R/1000;
	while(1)
	{
//		if ((ccd_edge_detect(0,127,225,img)==1))
//		{
//			SIU.GPDO[15].B.PDO=0;
//		}
		if((elec_flag==1))
		{
			PIT__stop(PIT_Timer1);
			LINFLEX_0.UARTCR.B.RXEN = 0;
			motor_output(motor_a[0],0);
			motor_output(motor_a[1],0);
			motor_output(motor_a[2],0);
			motor_output(motor_a[3],0);
			
			if(mode==WALL)
			{
				delay_ms(600);
//				if(destination[0][2]==destination[0][1])
//				{
//					PIT__config(PIT_Timer2,10,64,test,8);
//					start_time=STM.CNT.R/1000;
////					if(destination[0][0]-375<=0)
////					{
////						rotating_duty=-0.5;
////					}
////					else
////					{
////						rotating_duty=+0.5;
////					}
//					while(1)
//					{
//						if(finish_flag)
//						{
//							PIT__stop(PIT_Timer2);
//							break;
//						}
//					}
////					motor_output(motor_a[0],rotating_duty);
////					motor_output(motor_a[1],rotating_duty);
////					motor_output(motor_a[2],-rotating_duty);
////					motor_output(motor_a[3],-rotating_duty);
////					delay_ms(1150);
//					motor_output(motor_a[0],0);
//					motor_output(motor_a[1],0);
//					motor_output(motor_a[2],0);
//					motor_output(motor_a[3],0);
//					
//					SIU.GPDO[44].B.PDO=!(step%2);//姝ゅ杩樺簲鏈夌數纾侀搧鎿嶄綔
//					SIU.GPDO[40].B.PDO=!(step%2);
//					delay_ms(1000);
////					SIU.GPDO[44].B.PDO=!(step%2);
////					SIU.GPDO[40].B.PDO=!(step%2);
//					
//					motor_output(motor_a[0],-rotating_duty);
//					motor_output(motor_a[1],-rotating_duty);
//					motor_output(motor_a[2],rotating_duty);
//					motor_output(motor_a[3],rotating_duty);
//					delay_ms(1500);
//				}
//				
				SIU.GPDO[44].B.PDO=!(step)%2;//姝ゅ杩樺簲鏈夌數纾侀搧鎿嶄綔
				SIU.GPDO[40].B.PDO=!(step)%2;
				delay_ms(1000);
//				SIU.GPDO[44].B.PDO=!(step%2);
//				SIU.GPDO[40].B.PDO=!(step%2);
			}
			
			if(mode==CHESS)
			{
				SIU.GPDO[71].B.PDO=!(step%2);
				delay_ms(500);
				SIU.GPDO[45].B.PDO=step%2;
				delay_ms(500);
				SIU.GPDO[71].B.PDO=1;
				delay_ms(200);
				elec_flag=0;
				PIT__restart(PIT_Timer1);
				LINFLEX_0.UARTCR.B.RXEN = 1;
			}
			elec_flag=0;
			PIT__restart(PIT_Timer1);
			LINFLEX_0.UARTCR.B.RXEN = 1;
		}
		
		if(exit_flag)
		{
			PIT__stop(PIT_Timer1);
			LINFLEX_0.UARTCR.B.RXEN = 0;
			switch(direction)
			{
			case(LEFT):
				motor_output(motor_a[0],-0.5);
				motor_output(motor_a[1],0.5);
				motor_output(motor_a[2],-0.5);
				motor_output(motor_a[3],0.5);
			break;
			case(RIGHT):
				motor_output(motor_a[0],0.5);
				motor_output(motor_a[1],-0.5);
				motor_output(motor_a[2],0.5);
				motor_output(motor_a[3],-0.5);
			break;
			case(FORWARD):
				motor_output(motor_a[0],0.5);
				motor_output(motor_a[1],0.5);
				motor_output(motor_a[2],0.5);
				motor_output(motor_a[3],0.5);
			break;
			case(BEHIND):
				motor_output(motor_a[0],-0.5);
				motor_output(motor_a[1],-0.5);
				motor_output(motor_a[2],-0.5);
				motor_output(motor_a[3],-0.5);
			break;
			}
		}
	}
}

void Mode0_Quick(void)
{
	OLED_Fill(0x00);

	OLED_SetPointer(1,0);
	OLED_Str("v1 ");
	OLED_Float(motor_a[0]->y_distance);

	OLED_SetPointer(3,0);
	OLED_Str("v2 ");
	OLED_Float(delta);

	OLED_SetPointer(5,0);
	OLED_Str("v3 ");
	OLED_Float(motor_a[0]->actual_speed);
	
	OLED_SetPointer(7,0);
	OLED_Str("v4 ");
	OLED_Float(motor_a[0]->target_speed);
}

void test1()
{
	int duty_temp=0;
	float temp;
	Mode0_Quick();
	if((Start_Flag==0)||(step>=Step_Count)) 
	{
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		PIT__clear_flag(PIT_Timer1);
		return;
	}
	if(stop_flag)
	{
		step++;
        Target_D_Y=destination[1][step]-Y_location;
	    Target_D_X=destination[0][step]-X_location;
		stop_flag=0;
		elec_flag=1;
		i=0;
		j=0;
		g=0; //this is a try
		k=0; //this is a try
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		SIU.GPDO[12].B.PDO=1;
		if(step==Step_Count)
		{
			exit_flag=1;
		}
		PIT__clear_flag(PIT_Timer1);
		return;
	}
	SIU.GPDO[12].B.PDO=!SIU.GPDO[12].B.PDO;
	if(Send_Flag==1)
	{
		str[1]=sum_X;
		str[2]=sum_Y;
		delay_ms(500);
		BlueTx(str);
		Send_Flag=0;
	}
	
	//**********************************************************************//
	if(fabs(theta - 900)>80) omiga=-0.44;
	if((fabs(theta - 900)<=80)&&(fabs(theta - 900)>-80)) omiga=-0.55*(theta - 900)/100.0f; //姝ｄ负閫嗘椂閽堬紝璐熶负椤烘椂閽�
	if(fabs(theta - 900)<=-80) omiga=0.44;
	motor_a[0]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[1]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[2]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[3]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	
	//***************************瑙掑害鎺у埗****************************//  
	
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
	
	//鍧愭爣绯诲師鐐逛负宸︿笂瑙�	//************************璇诲彇杞﹂�***************************//
	if(run_flag)
	{
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
		Target_D_Y_R=destination[1][step]-Y_location+delta*100*(motor_a[0]->actual_speed);
		Y_location_R=Y_location-delta*100*(motor_a[0]->actual_speed);
	}
	else if((motor_a[0]->actual_speed)*(motor_a[1]->actual_speed)<0)
	{
		Target_D_X_R=destination[0][step]-X_location-delta*100*(motor_a[0]->actual_speed);
		X_location_R=X_location+delta*100*(motor_a[0]->actual_speed);
	}
	else if(motor_a[0]->actual_speed==0)
	{
		Target_D_Y_R=destination[1][step]-Y_location;
		Target_D_X_R=destination[0][step]-X_location;
	}//姝ゅ鍙兘鏈夌敤
	
	//*************************鍗″皵鏇兼护娉�****************************//
	
//	if(delta<=1.5)
//	{
	if(fabs(Target_D_Y_R)>near_threshold)
	{
		if(Target_D_Y_R>far_threshold) //鍚�
		{
			if(i<=50)
			{
				quick_speed=0.4+0.25*i/50;
				i++;
			}
			motor_a[0]->target_speed=-quick_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=-quick_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=-quick_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=-quick_speed+motor_a[3]->angel_speed;
		}
		else if((Target_D_Y_R<=far_threshold)&&(Target_D_Y_R>near_threshold)) 
		{
			if(j<=30)
			{
				slow_speed=0.65-0.35*j/30;
				j++;
			}
			motor_a[0]->target_speed=-slow_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=-slow_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=-slow_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=-slow_speed+motor_a[3]->angel_speed;
		}
		
		else if(Target_D_Y_R<=-far_threshold) //鍓�	{
		{	
			if(i<=50)
			{
				quick_speed=0.4+0.25*i/50;
				i++;
			}
			motor_a[0]->target_speed=quick_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=quick_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=quick_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=quick_speed+motor_a[3]->angel_speed;
		}
		else if((Target_D_Y_R<=-near_threshold)&&(Target_D_Y_R>-far_threshold)) 
		{
			if(j<=30)
			{
				slow_speed=0.65-0.35*j/30;
				j++;
			}
			motor_a[0]->target_speed=slow_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=slow_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=slow_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=slow_speed+motor_a[3]->angel_speed;
		}
	}
	else
	{
		if(Target_D_X_R>far_threshold)  //鍙�
		{
			if(k<=50)
			{
				quick_speed=0.4+0.25*k/50;
				k++;
			}
			motor_a[0]->target_speed=quick_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=-quick_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=quick_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=-quick_speed+motor_a[3]->angel_speed;
		}
		else if((Target_D_X_R<=far_threshold)&&(Target_D_X_R>near_threshold))  
		{
			if(j<=30)
			{
				slow_speed=0.65-0.35*j/30;
				j++;
			}
			motor_a[0]->target_speed=slow_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=-slow_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=slow_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=-slow_speed+motor_a[3]->angel_speed;
		}
		else if(Target_D_X_R<=-far_threshold)  //宸�{
		{
			if(g<=50)
			{
				quick_speed=0.4+0.25*g/50;
				g++;
			}
			motor_a[0]->target_speed=-quick_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=quick_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=-quick_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=quick_speed+motor_a[3]->angel_speed;
		}
		else if((Target_D_X_R<=-near_threshold)&&(Target_D_X_R>-far_threshold))  
		{
			if(j<=30)
			{
				slow_speed=0.65-0.35*j/30;
				j++;
			}
			motor_a[0]->target_speed=-slow_speed-motor_a[0]->angel_speed;
			motor_a[1]->target_speed=slow_speed-motor_a[1]->angel_speed;
			motor_a[2]->target_speed=-slow_speed+motor_a[2]->angel_speed;
			motor_a[3]->target_speed=slow_speed+motor_a[3]->angel_speed;
		}
//			else
//			{
//				motor_output(motor_a[0],0);
//				motor_output(motor_a[1],0);
//				motor_output(motor_a[2],0);
//				motor_output(motor_a[3],0);
//				PIT__clear_flag(PIT_Timer1);
//				return;
//			}
		}
//	}
	
	//***********************閫熷害鎺у埗******************************//
	
//	if(delta>=1.5)
//	{
//		if((X_location_R<15))
//		{
//			motor_a[0]->target_speed=0.4;
//			motor_a[1]->target_speed=-0.4;
//			motor_a[2]->target_speed=0.4;
//			motor_a[3]->target_speed=-0.4;
//		}
//		else if((X_location_R>385))
//		{
//			motor_a[0]->target_speed=-0.4;
//			motor_a[1]->target_speed=0.4;
//			motor_a[2]->target_speed=-0.4;
//			motor_a[3]->target_speed=0.4;
//		}
//		else if((Y_location_R<15))
//		{
//			motor_a[0]->target_speed=-0.4;
//			motor_a[1]->target_speed=-0.4;
//			motor_a[2]->target_speed=-0.4;
//			motor_a[3]->target_speed=-0.4;			
//		}
//		else if((Y_location_R>385))
//		{
//			motor_a[0]->target_speed=0.4;
//			motor_a[1]->target_speed=0.4;
//			motor_a[2]->target_speed=0.4;
//			motor_a[3]->target_speed=0.4;				
//		}
//	}
//
		
	if((fabs(Target_D_X_R)<=near_threshold)&&(fabs(Target_D_Y_R)<=near_threshold))//姝ゅ鍙兘鏈夌敤
	{
		stop_flag=1;
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		PIT__clear_flag(PIT_Timer1);
		return;
	}
	}	
//	
//	// if(die_flag)
//	// {
//	// 	motor_a[0]->target_speed=0.25;
//	// 	motor_a[1]->target_speed=0.25;
//	// 	motor_a[2]->target_speed=0.25;
//	// 	motor_a[3]->target_speed=0.25;
//	// }

	//****************************闃插嚭鐣屾帶鍒�********************************//
	
	PID__config(&motor_a[0]->motor_pid,0.24f,1.25f,0,10,10,70,10); //   0.16p 1.00i  //  0.64p 1.25i
	PID__config(&motor_a[1]->motor_pid,0.24f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.24f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.24f,1.25f,0,10,10,70,10);
	
	//*****************************PID閰嶇疆************************************//
	
	if((motor_a[0]->target_speed)*(motor_a[1]->target_speed)<0)
	{
		motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed);
		motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed);
		motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed);
		motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed);
	}
	else
	{
		motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed);
		motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed);
		motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed);
		motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed);	
	}
	
	//*****************************PID鎺у埗*************************************//
	
	if(motor_a[0]->duty>1.0f) motor_a[0]->duty=1.0f;
	if(motor_a[0]->duty<-1.0f) motor_a[0]->duty=-1.0f;
	if(motor_a[1]->duty>1.0f) motor_a[1]->duty=1.0f;
	if(motor_a[1]->duty<-1.0f) motor_a[1]->duty=-1.0f;
	if(motor_a[2]->duty>1.0f) motor_a[2]->duty=1.0f;
	if(motor_a[2]->duty<-1.0f) motor_a[2]->duty=-1.0f;
	if(motor_a[3]->duty>1.0f) motor_a[3]->duty=1.0f;
	if(motor_a[3]->duty<-1.0f) motor_a[3]->duty=-1.0f;
	
	//***************************杞﹂�闄愬箙***********************************//
	
	motor_output(motor_a[0],motor_a[0]->duty);
	motor_output(motor_a[1],motor_a[1]->duty);
	motor_output(motor_a[2],motor_a[2]->duty);
	motor_output(motor_a[3],motor_a[3]->duty);
	
	//*************************杈撳嚭PWM***************************//
	
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
	PIT__clear_flag(PIT_Timer1);
}

void test()
{
	int duty_temp=0;
	float duty_temp_1;
	int i=0;
	float rotating_time;
//	char str[3];
	Mode0_Quick();
	PID__config(&motor_a[0]->motor_pid,0.64f,1.25f,0,10,10,70,10); //1.44p 50i
	PID__config(&motor_a[1]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.64f,1.25f,0,10,10,70,10);
	if(destination[0][2]-X_location<=0)
	{
		rotating_duty=-0.4;
		rotating_time=2800;
	}
	else
	{
		rotating_duty=0.4;
		rotating_time=2800;
	}
	
	motor_a[0]->target_speed=rotating_duty;
	motor_a[1]->target_speed=rotating_duty;
	motor_a[2]->target_speed=-rotating_duty;
	motor_a[3]->target_speed=-rotating_duty;
//	Target_D=1.0f;
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
	motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed);
	motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed);
	motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed);
	motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed);
	if(motor_a[0]->duty>1.0f) motor_a[0]->duty=1.0f;
	if(motor_a[0]->duty<-1.0f) motor_a[0]->duty=-1.0f;
	if(motor_a[1]->duty>1.0f) motor_a[1]->duty=1.0f;
	if(motor_a[1]->duty<-1.0f) motor_a[1]->duty=-1.0f;
	if(motor_a[2]->duty>1.0f) motor_a[2]->duty=1.0f;
	if(motor_a[2]->duty<-1.0f) motor_a[2]->duty=-1.0f;
	if(motor_a[3]->duty>1.0f) motor_a[3]->duty=1.0f;
	if(motor_a[3]->duty<-1.0f) motor_a[3]->duty=-1.0f;
	
	stop_time=STM.CNT.R/1000;
	
	if(stop_time-start_time<=rotating_time)
	{
		motor_output(motor_a[0],motor_a[0]->duty);
		motor_output(motor_a[1],motor_a[1]->duty);
		motor_output(motor_a[2],motor_a[2]->duty);
		motor_output(motor_a[3],motor_a[3]->duty);
	}
//	else
//	{
//		motor_output(motor_a[0],0);
//		motor_output(motor_a[1],0);
//		motor_output(motor_a[2],0);
//		motor_output(motor_a[3],0);
//	}
	else
	{
		motor_output(motor_a[0],0);
		motor_output(motor_a[1],0);
		motor_output(motor_a[2],0);
		motor_output(motor_a[3],0);
		finish_flag=1;
		PIT__clear_flag(PIT_Timer2);
		return;
	}
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
	PIT__clear_flag(PIT_Timer2);
}

void test2()
{
	Mode0_Quick();
//	time=STM.CNT.R;
//	
//	if(time<last_time)
//	{
//		time_0=time+0xffffffff-last_time;
//	}
//	else
//	{
//		time_0=time-last_time;
//	}
	
//	delta=time_0/1000000.0f;
	motor_a[0]->y_distance+=0.01*100*motor_a[0]->actual_speed;
	PID__config(&motor_a[0]->motor_pid,0.24f,1.25f,0,10,10,70,10); //1.44p 1.25i
	PID__config(&motor_a[1]->motor_pid,0.24f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[2]->motor_pid,0.24f,1.25f,0,10,10,70,10);
	PID__config(&motor_a[3]->motor_pid,0.24f,1.25f,0,10,10,70,10);
	if(motor_a[0]->y_distance<100)
	{
	motor_a[0]->target_speed=0.65;
	motor_a[1]->target_speed=0.65;
	motor_a[2]->target_speed=0.65;
	motor_a[3]->target_speed=0.65;
	}
	if(motor_a[0]->y_distance>=100)
	{
		motor_a[0]->target_speed=0;
		motor_a[1]->target_speed=0;
		motor_a[2]->target_speed=0;
		motor_a[3]->target_speed=0;
	}
//	Target_D=1.0f;
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
	motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed);
	motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed);
	motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed);
	motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed);
	if(motor_a[0]->duty>1.0f) motor_a[0]->duty=1.0f;
	if(motor_a[0]->duty<-1.0f) motor_a[0]->duty=-1.0f;
	if(motor_a[1]->duty>1.0f) motor_a[1]->duty=1.0f;
	if(motor_a[1]->duty<-1.0f) motor_a[1]->duty=-1.0f;
	if(motor_a[2]->duty>1.0f) motor_a[2]->duty=1.0f;
	if(motor_a[2]->duty<-1.0f) motor_a[2]->duty=-1.0f;
	if(motor_a[3]->duty>1.0f) motor_a[3]->duty=1.0f;
	if(motor_a[3]->duty<-1.0f) motor_a[3]->duty=-1.0f;
		motor_output(motor_a[0], motor_a[0]->duty);
		motor_output(motor_a[1], motor_a[1]->duty);
		motor_output(motor_a[2], motor_a[2]->duty);
		motor_output(motor_a[3], motor_a[3]->duty);
	PIT__clear_flag(PIT_Timer1);
}
