/*
 * action.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dell
 */

#include "control.h"
#include "pwm.h" 
void vertical(struct motor t[],uint8_t duty_y) //向前为正，向后为负
{
	int i=0;
	t[0].duty = duty_y;
	t[1].duty = duty_y;
	t[2].duty = duty_y;
	t[3].duty = duty_y;
	for(i=0;i<4;i++)
	{
		if(t[i].duty>=1) t[i].duty = 1.0f;
		if(t[i].duty<=-1) t[i].duty = -1.0f;
		motor_output(t[i],t[i].duty);
	}
}

void horizontal(struct motor t[],uint8_t duty_x)  //向右为正，向左为负
{
	int i=0;
	t[0].duty = duty_x;
	t[1].duty = -duty_x;
	t[2].duty = -duty_x;
	t[3].duty = duty_x;
	for(i=0;i<4;i++)
	{
		if(t[i].duty>=1) t[i].duty = 1.0f;
		if(t[i].duty<=-1) t[i].duty = -1.0f;
		motor_output(t[i],t[i].duty);
	}
}

void rotating(struct motor t[],uint8_t angular_duty)  //顺时针为正，逆时针为负
{
	int i=0;
	t[0].duty = -angular_duty * (half_track_distance + half_wheelbase);
	t[1].duty = angular_duty * (half_track_distance + half_wheelbase);
	t[2].duty = -angular_duty * (half_track_distance + half_wheelbase);
	t[3].duty = angular_duty * (half_track_distance + half_wheelbase);
	for(i=0;i<4;i++)
	{
		if(t[i].duty>=1) t[i].duty = 1.0f;
		if(t[i].duty<=-1) t[i].duty = -1.0f;
		motor_output(t[i],t[i].duty);
	}
}

void motor_config(struct motor motor[],EMIOSn_CH forward_ch[],EMIOSn_CH backward_ch[])
{
	motor[0].backward_ch = backward_ch[0];
	motor[0].forward_ch = forward_ch[0];
	motor[1].backward_ch = backward_ch[1];
	motor[1].forward_ch = forward_ch[1];
	motor[2].backward_ch = backward_ch[2];
	motor[2].forward_ch = forward_ch[2];
	motor[3].backward_ch = backward_ch[3];
	motor[3].forward_ch = forward_ch[3];
	pwm__config(forward_ch[0]);
	pwm__config(backward_ch[0]);
	pwm__config(forward_ch[1]);
	pwm__config(backward_ch[1]);
	pwm__config(forward_ch[2]);
	pwm__config(backward_ch[2]);
	pwm__config(forward_ch[3]);
	pwm__config(backward_ch[3]);
	/**EMIOS9用于驱动板的pwm3通道输出持续高电平**/
	pwm__config(EMIOS_CH9);
	pwm__duty_update(EMIOS_CH9,1.0f);
}

void motor_output(struct motor motor , float duty)
{
	if(duty > 0.09)
	{
		pwm__duty_update(motor.forward_ch,duty);
		pwm__duty_update(motor.backward_ch,0);
	}
	if(duty < -0.09)
	{
		pwm__duty_update(motor.forward_ch,0);
		pwm__duty_update(motor.backward_ch,duty);
	}
	else
	{
		pwm__duty_update(motor.backward_ch,0);
		pwm__duty_update(motor.forward_ch,0);
	}
}


