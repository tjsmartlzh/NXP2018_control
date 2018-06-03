/*
 * uart.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#include"uart.h"
#include"stdlib.h"
#include "IntcInterrupts.h"
#include "gpio.h"
#include "math.h"
float X_location,Y_location,X_last_location,X_last_last_location,X_error,X_last_error,theta;  
uint8_t data[4];
uint8_t pramdata[32]; //增加数组长度，扩展通信协议
int     points[7];
uint8_t flagR=0,flagr=0,flagRr=0,Reverse_flag=0,Switch_lane_flag = 0,Switch_lane_trigger;
uint8_t FLAG;
uint8_t lane_flag,Start_line_flag;//赛道标志,用于设置目标车速
float   pram[4];
uint8_t Lampe_test;
extern int ccd_threshold;
extern uint8_t Reverse_finish;
uint8_t Elec_flag;
float destination[2][50];
uint8_t Start_Flag=0,Send_Flag=0;
int Step_Count=0,Step_Count_R,step=0;
float Target_D_X,Target_D_Y;
int sum_temp=0;
//extern uint8_t RC__flag;
int C_flag;
void LINFlex_TX(unsigned char data)
{
	LINFLEX_0.BDRL.B.DATA0 = data;       //发送语句
	while(!LINFLEX_0.UARTSR.B.DTF){}
	LINFLEX_0.UARTSR.B.DTF = 1;
}
//void UARTitosTX (int n)
//{
//  int i,j,sign;
//  char s[10];
//  if((sign=n)<0)//记录符号
//  n=-n;//使n成为正数
//  i=0;
//  do{
//       s[i++]=n%10+'0';//取下一个数字
//  }
//  while ((n/=10)>0);//删除该数字
//  if(sign<0)
//  s[i++]='-';
//  s[i]='\0';
//  for(j=i;j>=0;j--)//生成的数字是逆序的，所以要逆序输出
//	  LINFlex_TX(s[j]);
//}

//char* Int_to_char(int n)
//{
//	int i,j,sign;
//	char temp[10];
//	char s[10];
//	if((sign=n)<0)//记录符号
//		n=-n;//使n成为正数
//	i=0;
//	do{
//		temp[i++]=n%10+'0';//取下一个数字
//	}
//	while ((n/=10)>0);//删除该数字
//	if(sign<0)
//		temp[i++]='-';
//	temp[i]='\0';
//	for(j = 0;j<i;j++)
//		s[j] = temp[i-j-1];
//	s[j] = '\0';
//	return s;
//}
void f2s(float f, char* str)
{
	
    uint8_t i = 0,j=0;
    char temp[8];
    int n = (int)f;
    f = ((int)(f*100+0.5))/100.0f;//将f进行四舍五入保留两位小数
    f -= n;
    while (n > 0) {
        temp[i++] = n % 10 + '0';
        n /= 10;
    }
//    reverse(str,i);
	for(j = 0;j<i;j++)
		str[j] = temp[i-j-1];
	str[j] = '\0';
	
    str[i++] = '.';
    n = 0;
    while (f > 0 && n < 6) {
        int t = (int)(f * 10);
        str[i++] = t + '0';
        f = f * 10 - t;
        n++;
    }
    str[i] = '\0';
}



void BlueTx(char *send)                             //蓝牙发数据
{
	while(*send!=0x00)
	LINFlex_TX(*send++);
}
void Blue3Tx(char *send)
{
	while(*send!=0x00)
	LINFlex_3TX(*send++);
}
void LINFlex_3RX(void)
{
	uint8_t temp;
	data[0]=LINFLEX_3.BDRM.B.DATA4;        	// 读取接收到的数据
	data[1]=LINFLEX_3.BDRM.B.DATA5;			//
	data[2]=LINFLEX_3.BDRM.B.DATA6;		 //此版本必须每次正好发3Byte字节，否则读取的顺序有误
	data[3]=LINFLEX_3.BDRM.B.DATA7;	
	temp=data[3];
	LINFLEX_3.UARTSR.B.DRF = 1; 
}
void LINFlex_3TX(unsigned char data)
{
	LINFLEX_3.BDRL.B.DATA0 = data;       //发送语句
	while(!LINFLEX_3.UARTSR.B.DTF){}
	LINFLEX_3.UARTSR.B.DTF = 1;
}
void LINFlex_RX(void)
{
	uint8_t temp;
	data[0]=LINFLEX_0.BDRM.B.DATA4;        	// 读取接收到的数据
	data[1]=LINFLEX_0.BDRM.B.DATA5;			//
	data[2]=LINFLEX_0.BDRM.B.DATA6;		 //此版本必须每次正好发3Byte字节，否则读取的顺序有误
	data[3]=LINFLEX_0.BDRM.B.DATA7;	
	temp=data[3];
	switch (temp)
	{
	case 'X':
//		points[2]=data[2]-'0';    //调试换算
		points[2]=data[1]*256+data[2];    //实际换算
		GPIO__output__enable(13);
		SIU.GPDO[13].B.PDO=!SIU.GPDO[13].B.PDO;
		destination[0][Step_Count]=(points[2]-1)*50+25; //单位待定
	break;
	case 'Y': 
//		points[5]=data[2]-'0';    //调试换算
		points[5]=data[1]*256+data[2];    //实际换算
		sum_temp+=points[5];
		destination[1][Step_Count]=(points[5]-1)*50+25; //单位待定
		Step_Count++;
		if(Step_Count==Step_Count_R)
		{
			Start_Flag=1;
			Send_Flag=1;
		}
	break;
	case 'N': //是否执行吸棋子操作的标志位
//		pramdata[2]=data[2]-'0';    //调试换算
//		pram[0]=pramdata[2];    //调试换算
		pram[0]=data[1]*256+data[2];    //实际换算
		Step_Count_R=pram[0];
	break;
	case 'x':
//		points[0]=data[0]-'0';    //调试换算
//		points[1]=data[1]-'0';    //调试换算
//		points[2]=data[2]-'0';    //调试换算
//		X_location=points[0]*100+points[1]*10+points[2];    //调试换算
		X_location=data[1]*256+data[2];    //实际换算
		X_error=X_last_location-X_location;
		X_last_location=X_location;
		X_last_error=X_last_last_location-X_last_location;
		X_last_last_location=X_last_location;
		Target_D_X=destination[0][step]-X_location;
		C_flag=((fabs(X_error)-3)&&(fabs(X_last_error)-3));	
		
	break;
	case 'y': 
//		points[3]=data[0]-'0';    //调试换算
//		points[4]=data[1]-'0';	  //调试换算
//		points[5]=data[2]-'0';    //调试换算
//		Y_location=points[3]*100+points[4]*10+points[5];   //调试换算
		Y_location=data[1]*256+data[2];    //实际换算
		Target_D_Y=destination[1][step]-Y_location;

		if((fabs(Target_D_X)<=5)&&(fabs(Target_D_Y)<=5)&&(step<Step_Count))
		{
			step++;
		}
	break;

	case 'a': 
		theta=data[1]*256+data[2];    //实际换算
	break;
	case 'E':
		Elec_flag=1;
	break;
	default:
		initLINFlex_0_UART(12);
	break;
	}
	if (flagR||flagr) flagRr=1;
	LINFLEX_0.UARTSR.B.DRF = 1;  
	FLAG = 1;	//设置串口更新的标志
}

void initLINFlex_0_UART(uint8_t pri)
{
//配置LINFlex
  LINFLEX_0.LINCR1.B.INIT   = 1;   // 请求初始化
  LINFLEX_0.LINCR1.B.SLEEP  = 0;  // 禁止睡眠模式
  LINFLEX_0.LINCR1.B.BF     = 0;  // 如果ID不匹配不产生中断

  LINFLEX_0.UARTCR.B.UART   = 1;        // 进入UART模式
  LINFLEX_0.UARTCR.B.RXEN   = 1;   // 允许接收
  LINFLEX_0.UARTCR.B.TXEN   = 1;   // 允许发送
  LINFLEX_0.UARTCR.B.WL     = 1;        // 8位数据位
//  LINFLEX_0.UARTCR.B.OP     = 1;      // 偶校验
  LINFLEX_0.UARTCR.B.PCE    = 0;  // 禁止奇偶校验
  LINFLEX_0.UARTCR.B.TDFL   = 0;        // 发送缓冲区为1个字节
  LINFLEX_0.UARTCR.B.RDFL   = 3;        // 接收缓冲区为4个字节

  LINFLEX_0.LINIBRR.B.DIV_M = 416;      // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz
  LINFLEX_0.LINFBRR.B.DIV_F = 11;       // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz

//配置中断，使能中断功能
  LINFLEX_0.LINIER.B.DRIE   = 1;   // 数据接收完成中断
  LINFLEX_0.UARTSR.B.DRF    = 1;   // 清除接收完成标志
  LINFLEX_0.UARTSR.B.DTF    = 1;   // 清除发送完成标志
  
  LINFLEX_0.LINCR1.B.INIT   = 0;  // 变为正常模式
  
  SIU.PCR[18].R = 0x0400;    /* MPC56xxB: Configure port B2 as LIN0TX */
  SIU.PCR[19].R = 0x0103;    /* MPC56xxB: Configure port B3 as LIN0RX */
  INTC_InstallINTCInterruptHandler(LINFlex_RX,79,pri); 
}
//void initLINFlex_3_UART(uint8_t pri)
//{
//	//配置LINFlex
//	LINFLEX_3.LINCR1.B.INIT   = 1;   // 请求初始化
//	LINFLEX_3.LINCR1.B.SLEEP  = 0;  // 禁止睡眠模式
//	LINFLEX_3.LINCR1.B.BF     = 0;  // 如果ID不匹配不产生中断
//	
//	LINFLEX_3.UARTCR.B.UART   = 1;        // 进入UART模式
//	LINFLEX_3.UARTCR.B.RXEN   = 1;   // 允许接收
//	LINFLEX_3.UARTCR.B.TXEN   = 1;   // 允许发送
//	LINFLEX_3.UARTCR.B.WL     = 1;        // 8位数据位
//	//  LINFLEX_3.UARTCR.B.OP     = 1;      // 偶校验
//	LINFLEX_3.UARTCR.B.PCE    = 0;  // 禁止奇偶校验
//	LINFLEX_3.UARTCR.B.TDFL   = 0;        // 发送缓冲区为1个字节
//	LINFLEX_3.UARTCR.B.RDFL   = 3;        // 接收缓冲区为4个字节
//
//	LINFLEX_3.LINIBRR.B.DIV_M = 416;      // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz
//	LINFLEX_3.LINFBRR.B.DIV_F = 11;       // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz
//
//	//配置中断，使能中断功能
//	LINFLEX_3.LINIER.B.DRIE   = 1;   // 数据接收完成中断
//	LINFLEX_3.UARTSR.B.DRF    = 1;   // 清除接收完成标志
//	LINFLEX_3.UARTSR.B.DTF    = 1;   // 清除发送完成标志
//
//	LINFLEX_3.LINCR1.B.INIT   = 0;  // 变为正常模式
//
//	SIU.PCR[74].R = 0x0400;    /* MPC56xxB: Configure port E10 as LIN3TX */
//	SIU.PCR[75].R = 0x0103;    /* MPC56xxB: Configure port E11 as LIN3RX */
//	INTC_InstallINTCInterruptHandler(LINFlex_3RX,122,pri); 
//}

