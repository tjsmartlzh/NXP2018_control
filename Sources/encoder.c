/*
 * encoder.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#include"MPC5604B.h"
#include"encoder.h"
#include"gpio.h"
void Encoder__config(Encoder__t encoder,EMIOSn_CH CH,float transmisson_ratio, float resolution, uint32_t time,float radius)
{
	encoder->ch = CH;
	encoder->_resolution = resolution;
	encoder->_transmission_ratio = transmisson_ratio;
	encoder->_time = time/1000.0f;
	encoder->_radius = radius;
}
void Encoder__init(Encoder__t encoder)
{		
	uint8_t temp=encoder->ch;
	EMIOS_0.CH[encoder->ch].CCR.B.MODE = 0x13;	/* Mode is MCB *///？？？输入pwm
	EMIOS_0.CH[encoder->ch].CCR.B.BSL = 0x3;	/* Use internal counter 这个是时基*/
	EMIOS_0.CH[encoder->ch].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[encoder->ch].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 1 */
	//EMIOS_0.CH[16].CCR.B.FREN = 0;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[encoder->ch].CCR.B.EDPOL=1;	/* Edge Select rising edge */
	EMIOS_0.CH[encoder->ch].CADR.R=0xffff; 
				/* (WORD)EMIOS_0.CH[24].CCNTR.R 数据寄存器 读那个方波的周期  他在不停地累加*/
	SIU.PCR[8].R = 0x0102; 
	SIU.PCR[64].R = 0x0102;
	SIU.PCR[60].R = 0x0102;
	SIU.PCR[0].R = 0x0102;//检测脉冲 lsb  下拉电阻，限流吧，io口读一个方波
//	SIU.PCR[65].R = 0x0102;	//检测相位 dir   io口读一个高低电平
	GPIO__input__enable(encoder->dir_pad,1,0,0);
}
float Speed__bekommen(Encoder__t encoder)
{
	int speed_0;//两次counter的差值
	if(EMIOS_0.CH[encoder->ch].CCNTR.R < (encoder->_last_counter))		
		speed_0 =EMIOS_0.CH[encoder->ch].CCNTR.R-(encoder->_last_counter)+65535;
	else 
		speed_0 = EMIOS_0.CH[encoder->ch].CCNTR.R-(encoder->_last_counter);
	encoder->_last_counter =EMIOS_0.CH[encoder->ch].CCNTR.R;
	encoder->_speed = speed_0/(encoder->_resolution)*2*3.14159f/(encoder->_time)*(encoder->_transmission_ratio)*(encoder->_radius);
	return(encoder->_speed);
}

unsigned char Dir__bekommen(Encoder__t encoder)
{   
	encoder->dir = GPIO__read(65);
//	encoder->dir = SIU.GPDI[encoder->dir_pad].B.PDI;
	return(encoder->dir);
}

