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
float Radius,X_distance,Y_distance,x_distance,y_distance; 
uint8_t data[4];
uint8_t pramdata[32]; //增加数组长度，扩展通信协议
int     points[7];
uint8_t flagR=0,flagr=0,flagRr=0,Reverse_flag=0,Switch_lane_flag = 0,Switch_lane_trigger;
uint8_t FLAG,electromagnet_FLAG;
uint8_t lane_flag,Start_line_flag;//赛道标志,用于设置目标车速
float   pram[4];
uint8_t Lampe_test;
extern int ccd_threshold;
extern uint8_t Reverse_finish;
//extern uint8_t RC__flag;
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
	GPIO__output__enable(13);
	SIU.GPDO[13].B.PDO=!SIU.GPDO[13].B.PDO;
	data[0]=LINFLEX_0.BDRM.B.DATA4;        	// 读取接收到的数据
	data[1]=LINFLEX_0.BDRM.B.DATA5;			//
	data[2]=LINFLEX_0.BDRM.B.DATA6;		 //此版本必须每次正好发3Byte字节，否则读取的顺序有误
	data[3]=LINFLEX_0.BDRM.B.DATA7;	
	temp=data[3];
	switch (temp)
	{
	case 'X':
		points[0]=data[0]-'0';
		points[1]=data[1]-'0';
		points[2]=data[2]-'0';
		points[6]=1;
//		GPIO__output__enable(13);
//		SIU.GPDO[13].B.PDO=0;
		X_distance=(points[0]*100+points[1]*10+points[2])/100; //单位待定
	break;
	case 'Y': 
		points[3]=data[0]-'0';
		points[4]=data[1]-'0';
		points[5]=data[2]-'0';
		Y_distance=(points[0]*100+points[1]*10+points[2])/100; //单位待定
	break;
	case 'N': //是否执行倒车操作的标志位
		electromagnet_FLAG=1;
	break;
	case 'x':
		points[0]=data[0]-'0';
		points[1]=data[1]-'0';
		points[2]=data[2]-'0';
		points[6]=0;
		x_distance=(points[0]*100+points[1]*10+points[2])/100;
	break;
	case 'y':
		points[0]=data[0]-'0';
		points[1]=data[1]-'0';
		points[2]=data[2]-'0';
		points[6]=0;
		y_distance=(points[0]*100+points[1]*10+points[2])/100;
		break;
	case 'R':
		Radius=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
		flagR=1;
	break;
	case 'r':
		Radius=-((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
		flagr=1;
	break;

	case 'A': 
		//K1
		pramdata[0]=data[0]-'0';
		pramdata[1]=data[1]-'0';
		pramdata[2]=data[2]-'0';
		pram[0]=((pramdata[0]*100+pramdata[1]*10+pramdata[2]));
	break;
	case 'V':
		//B1
		pramdata[3]=data[0]-'0';
		pramdata[4]=data[1]-'0';
		pramdata[5]=data[2]-'0';
		pram[1]=(((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'))/1000.0f);
	break;
	case 'C': 
		//K2
		pramdata[6]=data[0]-'0';
		pramdata[7]=data[1]-'0';
		pramdata[8]=data[2]-'0';
		pram[2]=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
	break;
	case 'D': 
		//B2
		pramdata[9]=data[0]-'0';
		pramdata[10]=data[1]-'0';
		pramdata[11]=data[2]-'0';
		pram[3]=((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
	break;
	case 'E':
		//K2
		pramdata[12]=data[0]-'0';
		pramdata[13]=data[1]-'0';
		pramdata[14]=data[2]-'0';
	break;
	case 'F':
		//B2
		pramdata[15]=data[0]-'0';
		pramdata[16]=data[1]-'0';
		pramdata[17]=data[2]-'0';
	break;
	case 'W'://用于修改目标距离
		pramdata[18] = data[0]-'0';
		pramdata[19] = data[1]-'0';
		pramdata[20] = data[2]-'0';
	break;
	case 'S'://用于修改目标速度
		pramdata[21] = data[0]-'0';
		pramdata[22] = data[1]-'0';
		pramdata[23] = data[2]-'0';
	break;
	case 'f'://用于判断赛道标志
		lane_flag = data[2];
	break;
	case 'G':
		//用于切换赛道的标志位      Switch_lane_trigger = G 向左切换赛道
		// 					   Switch_lane_trigger = g 向右切换赛道
		Switch_lane_trigger = data[3];
		Switch_lane_flag = data[2];
	break;
	case 'g':
		Switch_lane_trigger = data[3];
		Switch_lane_flag = data[2];
	break;
	case 'L':
		//用于测试Infomation_lampe
//		Lampe_test = data[2];
//		GPIO__output__enable(15);
//		if (Lampe_test=='1') 	   SIU.GPDO[15].B.PDO=0;//右赛道
//		else if (Lampe_test=='2')  SIU.GPDO[15].B.PDO=1;//左赛道
	break;
	case'c':
		//用于确定CCD的阈值
//        ccd_threshold = ((data[0]-'0')*100+(data[1]-'0')*10+(data[2]-'0'));
    break;
	default:
		initLINFlex_0_UART(12);
		GPIO__output__enable(15);
		SIU.GPDO[15].B.PDO=0;
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
void initLINFlex_3_UART(uint8_t pri)
{
	//配置LINFlex
	LINFLEX_3.LINCR1.B.INIT   = 1;   // 请求初始化
	LINFLEX_3.LINCR1.B.SLEEP  = 0;  // 禁止睡眠模式
	LINFLEX_3.LINCR1.B.BF     = 0;  // 如果ID不匹配不产生中断
	
	LINFLEX_3.UARTCR.B.UART   = 1;        // 进入UART模式
	LINFLEX_3.UARTCR.B.RXEN   = 1;   // 允许接收
	LINFLEX_3.UARTCR.B.TXEN   = 1;   // 允许发送
	LINFLEX_3.UARTCR.B.WL     = 1;        // 8位数据位
	//  LINFLEX_3.UARTCR.B.OP     = 1;      // 偶校验
	LINFLEX_3.UARTCR.B.PCE    = 0;  // 禁止奇偶校验
	LINFLEX_3.UARTCR.B.TDFL   = 0;        // 发送缓冲区为1个字节
	LINFLEX_3.UARTCR.B.RDFL   = 3;        // 接收缓冲区为4个字节

	LINFLEX_3.LINIBRR.B.DIV_M = 416;      // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz
	LINFLEX_3.LINFBRR.B.DIV_F = 11;       // Baud Rate = 9600, In Case fipg_clock_lin = 64 MHz

	//配置中断，使能中断功能
	LINFLEX_3.LINIER.B.DRIE   = 1;   // 数据接收完成中断
	LINFLEX_3.UARTSR.B.DRF    = 1;   // 清除接收完成标志
	LINFLEX_3.UARTSR.B.DTF    = 1;   // 清除发送完成标志

	LINFLEX_3.LINCR1.B.INIT   = 0;  // 变为正常模式

	SIU.PCR[74].R = 0x0400;    /* MPC56xxB: Configure port E10 as LIN3TX */
	SIU.PCR[75].R = 0x0103;    /* MPC56xxB: Configure port E11 as LIN3RX */
	INTC_InstallINTCInterruptHandler(LINFlex_3RX,122,pri); 
}

