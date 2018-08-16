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
int straight_flag=1;
extern C_flag;
extern uint32_t time,last_time;
static uint32_t start_time,stop_time;
uint32_t time_0;
float delta;
float Target_D_X_R=0,Target_D_Y_R=0;
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
static int X_converse;
extern int CCD_count;

#define half_track_dis 0.21 
#define half_wheel_dis 0.16 
#define far_threshold 60
#define near_threshold 7

void Mode0_Quick(void);
void speed_control(void);
int main(void) 
{
	initModesAndClock(); 
	enableIrq();
	disableWatchdog();
	initLINFlex_0_UART(11);
	OLED_Init();
	STM_init();
//	 ccd_init();
	
	motor_config(&(Motor[0]),EMIOS_CH3,EMIOS_CH4,0,0,0,10,10,10,10);  //B11,B12
	motor_config(&(Motor[1]),EMIOS_CH5,EMIOS_CH6,0,0,0,10,10,10,10);  //B13,B14
	motor_config(&(Motor[2]),EMIOS_CH18,EMIOS_CH20,0,0,0,10,10,10,10);  //E2,E4
	motor_config(&(Motor[3]),EMIOS_CH21,EMIOS_CH22,0,0,0,10,10,10,10);  //E5,E6
	motor_a[0]=&(Motor[0]);
	motor_a[1]=&(Motor[1]);
	motor_a[2]=&(Motor[2]);
	motor_a[3]=&(Motor[3]);
	Encoder__config(&ecd[0],EMIOS_CH8,1,390,10,0.05,48);  //A8,D0(A2)
	Encoder__config(&ecd[1],EMIOS_CH24,1,390,10,0.05,52);  //D12,D4(A1)
	Encoder__config(&ecd[2],EMIOS_CH16,1,390,10,0.05,47);  //E0,C15
	Encoder__config(&ecd[3],EMIOS_CH0,1,390,10,0.05,41);  //A0,C9鍘熸湰鏄埖鏈虹殑鏃跺熀锛屾澶勬殏涓嶈�铏戣埖鏈虹殑浣跨敤锛屾晠鏆傜敤鏉ュ仛杈撳叆妫�ￄ1�7?????????????
	Encoder__init(&ecd[0]);
	Encoder__init(&ecd[1]); 
	Encoder__init(&ecd[2]);
	Encoder__init(&ecd[3]);
	GPIO__output__enable(14);
	GPIO__output__enable(15);
	GPIO__output__enable(12);
	GPIO__output__enable(13);
	GPIO__output__enable(71);  //E7 鎺ㄦ媺鏈烘�?
	GPIO__output__enable(45);  //C13 鐢电閾?
	GPIO__output__enable(44);  //C12 鍙冲墠鏂圭數纾侀ￄ1�7?????????????
	GPIO__output__enable(40);  //C8 宸﹀墠鏂圭數纾侀ￄ1�7?????????????
	SIU.GPDO[45].B.PDO=0; //0
	// SIU.GPDO[71].B.PDO=1;
	 PIT__config(PIT_Timer1,10,64,test1,10);
	str[0]=0x66;
//	str[1]=0x55;
//	str[2]=0x44;
	str[3]=0xff;
//	start_time=STM.CNT.R/1000;
	while(1)
	{
//		if ((ccd_edge_detect(0,127,1000,img)==1))
//		{
////			PIT__stop(PIT_Timer0);
//			SIU.GPDO[15].B.PDO=0;
//		}                        
		if((elec_flag==1))
		{
			PIT__stop(PIT_Timer1);
			LINFLEX_0.LINIER.B.DRIE   = 0;
			motor_output(motor_a[0],0);
			motor_output(motor_a[1],0);
			motor_output(motor_a[2],0);
			motor_output(motor_a[3],0);
			
			if(destination[3][step-1]==1)
			{
					;
			}
			if(destination[3][step-1]==TAKING)
			{
				SIU.GPDO[71].B.PDO=0;
				delay_ms(500);
				SIU.GPDO[45].B.PDO=1;
				delay_ms(500);
				SIU.GPDO[71].B.PDO=1;
				delay_ms(200);
			}
			else if(destination[3][step-1]==PUTTING)
			{
				SIU.GPDO[71].B.PDO=1;
				delay_ms(500);
				SIU.GPDO[45].B.PDO=0;
				delay_ms(500);
			}
			else if(destination[3][step-1]==WALL)
			{
				delay_ms(600);			
				SIU.GPDO[44].B.PDO=!(step)%2;//姝ゅ杩樺簲鏈夌數纾侢�搧鎿嶄綔
				SIU.GPDO[40].B.PDO=!(step)%2;
				delay_ms(1000);
			}

			elec_flag=0;
			PIT__restart(PIT_Timer1);
			LINFLEX_0.LINIER.B.DRIE = 1;
		}
		
		if(exit_flag)
		{
			PIT__stop(PIT_Timer1);
			LINFLEX_0.LINIER.B.DRIE = 0;
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
	OLED_Float(fabs(Target_D_Y_R));

	OLED_SetPointer(3,0);
	OLED_Str("v2 ");
	OLED_Float(straight_flag);

//	OLED_SetPointer(5,0);
//	OLED_Str("v3 ");
//	OLED_Float(ccd_edge_detect(0,127,1000,img));
//	
//	OLED_SetPointer(7,0);
//	OLED_Str("v4 ");
//	OLED_Float(CCD_count);
}

void test1()
{
	int duty_temp=0;
	float temp;
	Mode0_Quick();
	if((Start_Flag==0)||(step>=Step_Count)) 
	{
		motor_a[0]->target_speed=0;
		motor_a[1]->target_speed=0;
		motor_a[2]->target_speed=0;
		motor_a[3]->target_speed=0;
		speed_control();
		PIT__clear_flag(PIT_Timer1);
		return;
	}
	if(stop_flag)
	{
		step++;
		OLED_SetPointer(5,0);
		OLED_Str("v3 ");
		OLED_Float(X_location);
	
		OLED_SetPointer(7,0);
		OLED_Str("v4 ");
		OLED_Float(Y_location);
//      Target_D_Y_R=destination[1][step]-Y_location;
//	    Target_D_X_R=destination[0][step]-X_location;
		stop_flag=0;
		elec_flag=1;
		straight_flag=1;
		X_converse=0;
		i=0;
		j=0;
		g=0; //this is a try
		k=0; //this is a try
		motor_a[0]->target_speed=0;
		motor_a[1]->target_speed=0;
		motor_a[2]->target_speed=0;
		motor_a[3]->target_speed=0;
		speed_control();
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
	if((fabs(theta - 900)<=80)&&(fabs(theta - 900)>-80)) omiga=-0.55*(theta - 900)/100.0f; //姝ｄ负閫嗘椂閽堬紝璐熶负椤烘椂閽?
	if(fabs(theta - 900)<=-80) omiga=0.44;
	motor_a[0]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[1]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[2]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	motor_a[3]->angel_speed=omiga*(half_track_dis + half_wheel_dis);
	
	//***************************瑙掑害鎺уￄ1�7?????????????****************************//  
	
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
	
	//鍧愭爣绯诲師鐐��负宸︿笂鏂?
	//************************璇诲彇杞﹂�***************************//
	
	// if((motor_a[0]->actual_speed<(-0.2)) && (motor_a[1]->actual_speed)<(-0.2))
	// {
	// 	Target_D_Y_R=destination[1][step]-Y_location-3;
	// 	Target_D_X_R=destination[0][step]-X_location;
	// }
	// else
	// {
	// 	if((motor_a[0]->actual_speed<0) && (motor_a[1]->actual_speed>0))
	// 	{
	// 		Target_D_Y_R=destination[1][step]-Y_location+8;
	// 		Target_D_X_R=destination[0][step]-X_location+3;
	// 	}
	// 	else if((motor_a[0]->actual_speed>0) && (motor_a[1]->actual_speed<0))
	// 	{
	// 		Target_D_Y_R=destination[1][step]-Y_location+8;
	// 		Target_D_X_R=destination[0][step]-X_location-3;
	// 	}
	// 	else
	// 	{
	// 		Target_D_Y_R=destination[1][step]-Y_location+8;
	// 		Target_D_X_R=destination[0][step]-X_location;
	// 	}
	// }
	
//����ΪС���Ӱ汾

		if((motor_a[0]->actual_speed<(-0.15)) && (motor_a[1]->actual_speed)<(-0.15))  //behind
		{
			Target_D_Y_R=destination[1][step]-Y_location-5;
			Target_D_X_R=destination[0][step]-X_location;
			// tiaozheng=1;
		}
		else
		{
			if((motor_a[0]->actual_speed<(-0.15)) && (motor_a[1]->actual_speed>=0.15)) //left
			{
				Target_D_Y_R=destination[1][step]-Y_location;
				Target_D_X_R=destination[0][step]-X_location+2;
				// tiaozheng=1;
			}
			else if((motor_a[0]->actual_speed>=0.15) && (motor_a[1]->actual_speed<(-0.15))) //right
			{
				Target_D_Y_R=destination[1][step]-Y_location;
				Target_D_X_R=destination[0][step]-X_location-2;
				// tiaozheng=1;
			}
			else
			{
				Target_D_Y_R=destination[1][step]-Y_location+5;
				Target_D_X_R=destination[0][step]-X_location;
				// tiaozheng=1;
			}
		}
//���Ǵ����Ӱ汾

	//*************************鍗��皵鏇兼护娉?****************************//
	
			if((fabs(Target_D_Y_R)>near_threshold) && (straight_flag==1))
			{
				if(Target_D_Y_R>far_threshold) //ￄ1�7??????????????
				{
					if(i<=80)
					{
						quick_speed=0.4+0.35*i/80;
						i++;
					}
					motor_a[0]->target_speed=-quick_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=-quick_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=-quick_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=-quick_speed+motor_a[3]->angel_speed;
					}
				else if((Target_D_Y_R<=far_threshold)&&(Target_D_Y_R>near_threshold)) 
				{
					if((destination[3][step]==WALL) && (Target_D_Y_R<=20))
					{
						ccd_init();
						// SIU.GPDO[15].B.PDO=!SIU.GPDO[15].B.PDO;
						if(ccd_edge_detect(0,127,1000,img))
						{
							straight_flag=0;
							SIU.GPDO[15].B.PDO=0;
							PIT__stop(PIT_Timer0);
						}
					}

					if(j<=50)
					{
						slow_speed=0.4-0.2*j/50;
						j++;
					}
					motor_a[0]->target_speed=-slow_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=-slow_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=-slow_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=-slow_speed+motor_a[3]->angel_speed;

				}
						
				else if(Target_D_Y_R<=-far_threshold) //ￄ1�7??????????????
				{	
					if(i<=80)
					{
						quick_speed=0.4+0.35*i/80;
						i++;
					}
					motor_a[0]->target_speed=quick_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=quick_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=quick_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=quick_speed+motor_a[3]->angel_speed;
				}
				else if((Target_D_Y_R<=-near_threshold)&&(Target_D_Y_R>-far_threshold)) 
				{
					if((destination[3][step]==WALL) && (Target_D_Y_R>=-20))
					{
						ccd_init();
					// SIU.GPDO[15].B.PDO=!SIU.GPDO[15].B.PDO;
						if(ccd_edge_detect(0,127,1000,img))
						{
							straight_flag=0;
							SIU.GPDO[15].B.PDO=0;
							PIT__stop(PIT_Timer0);
						}
					}

					if(j<=50)
			    	{
			 			slow_speed=0.4-0.2*j/50;
						j++;
					}
					motor_a[0]->target_speed=slow_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=slow_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=slow_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=slow_speed+motor_a[3]->angel_speed;
				}
			}
			else if((fabs(Target_D_Y_R)<=near_threshold) && (straight_flag==1))  straight_flag=0;
			else if(((fabs(Target_D_X_R)>near_threshold) && (straight_flag==0)) || (X_converse==1))
			{
				if(Target_D_X_R>far_threshold)  //ￄ1�7??????????????
				{
					if(k<=80)
					{
						quick_speed=0.4+0.35*k/80;
						k++;
					}
					motor_a[0]->target_speed=quick_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=-quick_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=quick_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=-quick_speed+motor_a[3]->angel_speed;
				}
				else if((Target_D_X_R<=far_threshold)&&(Target_D_X_R>near_threshold))  
				{
					if(j<=50)
					{
						slow_speed=0.4-0.2*j/50;
						j++;
					}
					motor_a[0]->target_speed=slow_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=-slow_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=slow_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=-slow_speed+motor_a[3]->angel_speed;
				}
				else if(Target_D_X_R<=-far_threshold)  //ￄ1�7??????????????
				{
					if(g<=80)
					{
						quick_speed=0.4+0.35*g/80;
						g++;
					}
					motor_a[0]->target_speed=-quick_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=quick_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=-quick_speed+motor_a[2]->angel_speed;
					motor_a[3]->target_speed=quick_speed+motor_a[3]->angel_speed;
				}
				else if((Target_D_X_R<=-near_threshold)&&(Target_D_X_R>-far_threshold))  
				{
					if(j<=50)
					{
						slow_speed=0.4-0.2*j/50;
						j++;
					}
					motor_a[0]->target_speed=-slow_speed-motor_a[0]->angel_speed;
					motor_a[1]->target_speed=slow_speed-motor_a[1]->angel_speed;
					motor_a[2]->target_speed=-slow_speed+motor_a[2]->angel_speed;
		    		motor_a[3]->target_speed=slow_speed+motor_a[3]->angel_speed;
				}
			}
			else if(((fabs(Target_D_Y_R)>near_threshold)&&(fabs(Target_D_X_R)<=near_threshold)&&(straight_flag==0)))
			{
				straight_flag=1;
			}
	
//С����0.4/0.45��0.65/0.35

		if((fabs(Target_D_X_R)<=near_threshold)&&(fabs(Target_D_Y_R)<=near_threshold)&&(step<Step_Count))//姝ゅ鍙兘鏈夌ￄ1�7?????????????
		{
			stop_flag=1;
//			 SIU.GPDO[15].B.PDO=!SIU.GPDO[15].B.PDO;
			// motor_output(motor_a[0],0);
			// motor_output(motor_a[1],0);
			// motor_output(motor_a[2],0);
			// motor_output(motor_a[3],0);
			motor_a[0]->target_speed=0;
			motor_a[1]->target_speed=0;
			motor_a[2]->target_speed=0;
			motor_a[3]->target_speed=0;
			speed_control();
			PIT__clear_flag(PIT_Timer1);
			return;
		}

		speed_control();
	
	SIU.GPDO[14].B.PDO=!SIU.GPDO[14].B.PDO;
	PIT__clear_flag(PIT_Timer1);
}

	void speed_control()
	{
			//***********************閫熷害鎺уￄ1�7?????????????******************************//
		PID__config(&motor_a[0]->motor_pid,0.24f,1.25f,0,10,10,70,10); //   0.16p 1.00i  //  0.64p 1.25i
		PID__config(&motor_a[1]->motor_pid,0.24f,1.25f,0,10,10,70,10);
		PID__config(&motor_a[2]->motor_pid,0.24f,1.25f,0,10,10,70,10);
		PID__config(&motor_a[3]->motor_pid,0.24f,1.25f,0,10,10,70,10);
		
		//*****************************PID閰嶇ￄ1�7?????????????************************************//
		
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
		
		//***************************杞﹂�闄愬�***********************************//
		
		motor_output(motor_a[0],motor_a[0]->duty);
		motor_output(motor_a[1],motor_a[1]->duty);
		motor_output(motor_a[2],motor_a[2]->duty);
		motor_output(motor_a[3],motor_a[3]->duty);
		
		//*************************杈撳嚭PWM***************************//
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
//	motor_a[0]->y_distance+=0.01*100*motor_a[0]->actual_speed;
//	PID__config(&motor_a[0]->motor_pid,0.24f,1.25f,0,10,10,70,10); //1.44p 1.25i
//	PID__config(&motor_a[1]->motor_pid,0.24f,1.25f,0,10,10,70,10);
//	PID__config(&motor_a[2]->motor_pid,0.24f,1.25f,0,10,10,70,10);
//	PID__config(&motor_a[3]->motor_pid,0.24f,1.25f,0,10,10,70,10);
//	if(motor_a[0]->y_distance<300)
//	{
//	motor_a[0]->target_speed=0.8;
//	motor_a[1]->target_speed=0.8;
//	motor_a[2]->target_speed=0.8;
//	motor_a[3]->target_speed=0.8;
//	}
//	if(motor_a[0]->y_distance>=300)
//	{
//		motor_a[0]->target_speed=0;
//		motor_a[1]->target_speed=0;
//		motor_a[2]->target_speed=0;
//		motor_a[3]->target_speed=0;
//	}
////	Target_D=1.0f;
//	Speed__bekommen(&ecd[0]);
//	if(Dir__bekommen(&ecd[0])) ecd[0]._speed=-(ecd[0]._speed);
//	Speed__bekommen(&ecd[1]);
//	if(Dir__bekommen(&ecd[1])) ecd[1]._speed=-(ecd[1]._speed);
//	Speed__bekommen(&ecd[2]);
//	if(Dir__bekommen(&ecd[2])) ecd[2]._speed=-(ecd[2]._speed);
//	Speed__bekommen(&ecd[3]);
//	if(Dir__bekommen(&ecd[3])) ecd[3]._speed=-(ecd[3]._speed);
//	motor_a[0]->actual_speed=ecd[0]._speed;
//	motor_a[1]->actual_speed=ecd[1]._speed;
//	motor_a[2]->actual_speed=ecd[2]._speed;
//	motor_a[3]->actual_speed=ecd[3]._speed;
//	motor_a[0]->duty+=PID__update(&(motor_a[0]->motor_pid), motor_a[0]->target_speed, motor_a[0]->actual_speed);
//	motor_a[1]->duty+=PID__update(&(motor_a[1]->motor_pid), motor_a[1]->target_speed, motor_a[1]->actual_speed);
//	motor_a[2]->duty+=PID__update(&(motor_a[2]->motor_pid), motor_a[2]->target_speed, motor_a[2]->actual_speed);
//	motor_a[3]->duty+=PID__update(&(motor_a[3]->motor_pid), motor_a[3]->target_speed, motor_a[3]->actual_speed);
//	if(motor_a[0]->duty>1.0f) motor_a[0]->duty=1.0f;
//	if(motor_a[0]->duty<-1.0f) motor_a[0]->duty=-1.0f;
//	if(motor_a[1]->duty>1.0f) motor_a[1]->duty=1.0f;
//	if(motor_a[1]->duty<-1.0f) motor_a[1]->duty=-1.0f;
//	if(motor_a[2]->duty>1.0f) motor_a[2]->duty=1.0f;
//	if(motor_a[2]->duty<-1.0f) motor_a[2]->duty=-1.0f;
//	if(motor_a[3]->duty>1.0f) motor_a[3]->duty=1.0f;
//	if(motor_a[3]->duty<-1.0f) motor_a[3]->duty=-1.0f;
		motor_output(motor_a[0], 0.6);
		motor_output(motor_a[1], 0.6);
		motor_output(motor_a[2], 0.6);
		motor_output(motor_a[3], 0.6);
	PIT__clear_flag(PIT_Timer1);
}
