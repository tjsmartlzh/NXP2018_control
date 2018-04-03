/*
 * pwm.h
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#ifndef PWM_H_
#define PWM_H_

#include"MPC5604B.h"
#include"stdbool.h"
typedef enum{
	 EMIOS_CH0 = 0,EMIOS_CH1 = 1,EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5,EMIOS_CH6,
	EMIOS_CH7,EMIOS_CH8,EMIOS_CH9,EMIOS_CH10,EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,
	EMIOS_CH14,EMIOS_CH15,EMIOS_CH16,EMIOS_CH17,EMIOS_CH18,EMIOS_CH19,EMIOS_CH20,
	EMIOS_CH21,EMIOS_CH22,EMIOS_CH23,EMIOS_CH24,EMIOS_CH25,EMIOS_CH26,EMIOS_CH27
}EMIOSn_CH;
//EMIOSn_CH forward_ch_r[4]={EMIOS_CH2,EMIOS_CH3,EMIOS_CH4,EMIOS_CH5};
//EMIOSn_CH backward_ch_r[4]={EMIOS_CH11,EMIOS_CH12,EMIOS_CH13,EMIOS_CH14};
void EMIOS__init(void);
void pwm__config(EMIOSn_CH ch);
void pwm__duty_update(EMIOSn_CH ch,float duty);
void pwm__input_measure_config(EMIOSn_CH ch);

#endif /* PWM_H_ */
