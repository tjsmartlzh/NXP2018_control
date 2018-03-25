/*
 * encoder.h
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include"MPC5604B.h"
#include"pwm.h"
typedef struct{
	EMIOSn_CH ch;   	//电磁组采用的EMIOS通道为具有MCB功能的ch16
	float _transmission_ratio;// 传动比 0.16515*29/28 = 0.17105// 新车0.15502
	float _radius;		//车轮半径 32mm？
	float _speed;		//车轮相对于地面的绝对速度
	float _resolution; 	//光编线数
	uint32_t _last_counter;
	float _time;		//采集周期
	uint8_t dir;		//车速的方向
	uint8_t dir_pad;
} ECD_t,*Encoder__t;
/**
 * @ _transmission_ratio 
 */
void Encoder__config(Encoder__t encoder,EMIOSn_CH CH,float transmisson_ratio, float resolution,
		uint32_t time,float radius,uint8_t dir_pad);
void Encoder__init(Encoder__t encoder);
float Speed__bekommen(Encoder__t encoder);
unsigned char Dir__bekommen(Encoder__t encoder);
unsigned char NullBekommen(void);


#endif /* ENCODER_H_ */
