/*
 * ccd_capture.c
 *
 *  Created on: Feb 1, 2018
 *      Author: SanqingQu
 */

#include "ccd_capture.h"
#include "MPC5604B.h"
#include "uart.h"
#include "oled.h"

int img[128];
unsigned char *send;
unsigned char putstring[]="Image";
int ccd_threshold = 200; //默认200 可以根据上位机进行调整该阈值              1.200 2.?
void Mode0_Quick(void);
void ccd_init()
{
	ADC.MCR.R = 0x20000000;       /*未读转换数据不能被覆盖；右对齐；连续转换序列模式；引起当前链转换结束并终止操作；对通道注入的外触发器使不能；模拟时钟频率为40MHz；CTU触发转换使不能；自动时钟关闭使不能；正常模式*/
	ADC.NCMR[1].R = 0x00000004;   /*使能CH34、33通道（标准通道）的位正常采样*/
	ADC.CTR[1].R = 0x00008606;    /*转换时间寄存器 与标准通道相关联*/
	ADC.MCR.B.NSTART=1;          /* Trigger normal conversions for ADC0 */
	SIU.PCR[26].R = 0x2100;     // 定义ccd的AO 端口    B10
	GPIO__output__enable(33);   // 定义ccd的SI 端口    C1
	GPIO__output__enable(36);   // 定义ccd的CLK端口    C4
	PIT__config(PIT_Timer0,5,64,ccd_capture,9);
//	 PIT__config(PIT_Timer2,10,64,ccd_threshold_detect,12);//用于阈值确定
}

void ccd_capture()
{
	int A0result;
	static int count;
	int i;
	CCDR_SI = 1;
	delay_us(1);
	CCDR_CLK = 1;
	delay_us(1);
	CCDR_SI = 0;
	delay_us(20);
	while (ADC.CDR[34].B.VALID != 1) {}; /* 等待最后一次扫描完成*/
	A0result = ADC.CDR[34].B.CDATA; /* 读取ANS1的转换结果数据*/
	img[0] = A0result;
	CCDR_CLK = 0;
	for(i = 0;i<127;i++)
	{
		delay_us(4);
		CCDR_CLK = 1;
		delay_us(4);
		while (ADC.CDR[34].B.VALID != 1) {}; /* 等待扫描完成*/
		A0result = ADC.CDR[34].B.CDATA; /* 读取ANS1的转换结果数据*/
		img[i+1] = A0result;
		CCDR_CLK = 0;
	}
	delay_us(4);
	CCDR_CLK = 1;
	delay_us(4);
	CCDR_CLK = 0;
	PIT__clear_flag(PIT_Timer0);

	count++;

//	if(count%3 == 0) {ccd_max_min_send(img);ccd_pix60pix100_send(img);}//调试使用，最后需要关闭
//	if(count%3 == 0) ccd_img_send();
}
/**
 * sta_pix 边缘检测的开始像素点; 经过测试 取sta_pix = 70
 * end_pix 边缘检测的结束像素点; 经过测试 取end_pix = 90 
 * int threshold CCD图像二值化的阈值; 实验室阈值 取150
 * int pic[] CCD图像数组 128*1;
 */
void ccd_threshold_detect()
{
	static ccd_send_flag;
	static int ccd_temp_img[128];
	char format[2] = ":";
	char char_space[3]=" ";
	uint8_t i;
	if(!ccd_send_flag)
		{
			for(i = 0;i<128;i++)ccd_temp_img[i] = img[i]; ccd_send_flag = 1;
		}
	if(ccd_send_flag)
	{
		for(i = 0;i<128;i++)
		{
			BlueTx(Int_to_char(i));
			BlueTx(format);
			if(i==0)
			{
				LINFlex_TX('B');
			}
			BlueTx(Int_to_char(ccd_temp_img[i]));
			BlueTx(char_space);
		}
		ccd_send_flag = 0;
	}
	ccd_max_min_send(ccd_temp_img);
	PIT__clear_flag(PIT_Timer2);
}
uint8_t ccd_edge_detect(uint8_t sta_pix,uint8_t end_pix,int threshold,int pic[])
{
	uint8_t i;
	uint8_t j;
	uint8_t temp=0;
	static uint8_t last,now;
	static int count;
	for(i = 0; i < 128; i++)
	{
		if(pic[i] >600)
		{
			return 0;
		}
	} //滤波
	for(i = 0; i < 128; i++)
	{
		//将CCD图像进行二值化
		if(pic[i] >= threshold)
			pic[i] = 1;
		else
			pic[i] = 0;	
	}
	for(i = 0; i <= 127; i++)if(pic[i])temp++;
//	OLED_Fill(0x00);
//	OLED_SetPointer(3,0);
//	OLED_Str("v3 ");
//	OLED_Float(pic[63]); //调试用
//	OLED_Fill(0x00);
//	OLED_SetPointer(7,0);
//	OLED_Str("v4 ");
//	OLED_Float(temp); //调试用
	if(temp >=125)  //end_pix-sta_pix
	{
		if (last) 
		 {
			last=1; return 1;
		 }
		else 
		 {
			last=1;return 0;
		 }
	} 
	else {last=0; return 0;}
//	if (count>=3)return 1; else return 0;//有障碍是1无障碍是0
}
void ccd_max_min_send(int pic[])
{
	char char_max[6] = "Max: ";
	char char_min[6] = "Min: ";
	char char_space[4] = "   ";
	int i = 0;
	int max_temp = 0, min_temp = pic[0];
	for(i;i<128;i++)
	{
		if(pic[i]>max_temp) max_temp = pic[i];
		if(pic[i]<min_temp) min_temp = pic[i];
	}
	BlueTx(char_max);
	BlueTx(Int_to_char(max_temp));
	BlueTx(char_space);
	BlueTx(char_min);
	BlueTx(Int_to_char(min_temp));
	BlueTx(char_space);
}
void ccd_pix60pix100_send(int pic[])
{
	char format[2] = ":";
	char char_space[3]="  ";
	int i = 0;
	for(i = 50;i<=90;i++)
	{
		BlueTx(Int_to_char(i));
		BlueTx(format);
		BlueTx(Int_to_char(pic[i]));
		BlueTx(char_space);
	}
}
//void ccd_img_send()
//{
//	unsigned char aa='L';
//	uint8_t Ts = 0,Tc = 0;
//	send = putstring;
//	LINFlex_TX(*send++);
//	for(Ts=0;Ts<5;)
//	{
//		switch(Ts){
//		case 0: if(*send!=0x00){
//			LINFlex_TX(*send++);
//			break;}
//		else{
//			Ts=1;
//			break;}
//		case 1:
//			LINFlex_TX(aa);
//			Ts=2;
//			break;
//		case 2: 
//			LINFlex_TX(SendInt2(img[Tc]));        //发送CCD图像
//			Ts=3;
//			break;
//		case 3: 
//			LINFlex_TX(SendInt3(img[Tc]));
//			Ts=4;
//			break;
//		case 4: 
//			LINFlex_TX(SendInt4(img[Tc]));
//			if(Tc<127){
//				Tc++;
//				Ts=2;}
//			else{
//				Tc=0;
//				Ts=5;}
//			break;
//		}
//	}
//}
