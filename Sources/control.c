/*
 * action.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dell
 */

#include "control.h"
//#include "pwm.h" 
//struct motor Motor[4];  //0->nw; 1->ne; 2->sw; 3->se
void vertical_output(motor t[],uint8_t speed_y) //��ǰΪ�������Ϊ��
{
	int i=0;
	t[0]->duty = speed_y;
	t[1]->duty = speed_y;
	t[2]->duty = speed_y;
	t[3]->duty = speed_y;
	for(i=0;i<4;i++)
	{
		if(t[i]->duty>=1) t[i]->duty = 1.0f;
		if(t[i]->duty<=-1) t[i]->duty = -1.0f;
		motor_output(t[i],t[i]->duty);
	}
}

void horizontal_output(motor t[],uint8_t speed_x)  //����Ϊ��������Ϊ��
{
	int i=0;
	t[0]->duty = speed_x;
	t[1]->duty = -speed_x;
	t[2]->duty = -speed_x;
	t[3]->duty = speed_x;
	for(i=0;i<4;i++)
	{
		if(t[i]->duty>=1) t[i]->duty = 1.0f;
		if(t[i]->duty<=-1) t[i]->duty = -1.0f;
		motor_output(t[i],t[i]->duty);
	}
}

void rotating_output(motor t[],uint8_t angular_speed)  //˳ʱ��Ϊ������ʱ��Ϊ��
{
	int i=0;
	t[0]->duty = -angular_speed * (half_track_distance + half_wheelbase);
	t[1]->duty = angular_speed * (half_track_distance + half_wheelbase);
	t[2]->duty = -angular_speed * (half_track_distance + half_wheelbase);
	t[3]->duty = angular_speed * (half_track_distance + half_wheelbase);
	for(i=0;i<4;i++)
	{
		if(t[i]->duty>=1) t[i]->duty = 1.0f;
		if(t[i]->duty<=-1) t[i]->duty = -1.0f;
		motor_output(t[i],t[i]->duty);
	}
}

void motor_config(motor t,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch)
{
	t->backward_ch = backward_ch;
	t->forward_ch = forward_ch;
	pwm__config(forward_ch);
	pwm__config(backward_ch);
	/**EMIOS9�����������pwm3ͨ����������ߵ�ƽ**/
	pwm__config(EMIOS_CH9);
	pwm__duty_update(EMIOS_CH9,1.0f);
}

void motor_output(motor t , float duty)
{
	if(duty > 0.09)
	{
		pwm__duty_update(t->forward_ch,duty);
		pwm__duty_update(t->backward_ch,0);
	}
	if(duty < -0.09)
	{
		pwm__duty_update(t->forward_ch,0);
		pwm__duty_update(t->backward_ch,duty);
	}
	else
	{
		pwm__duty_update(t->backward_ch,0);
		pwm__duty_update(t->forward_ch,0);
	}
}


