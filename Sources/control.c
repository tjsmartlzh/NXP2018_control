/*
 * action.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dell
 */

#include "control.h"
//#include "pwm.h" 
//struct motor *Motor;  //0->nw; 1->ne; 2->sw; 3->se
void vertical_output(Motor t[],float speed_y) //向前为正，向后为负
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

void horizontal_output(Motor t[],uint8_t speed_x)  //向右为正，向左为负
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

void rotating_output(Motor t[],uint8_t angular_speed)  //顺时针为正，逆时针为负
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

void motor_config(Motor t[],EMIOSn_CH forward_ch[],EMIOSn_CH backward_ch[])
{
	t[0]->backward_ch = backward_ch[0];
	t[0]->forward_ch = forward_ch[0];
	t[1]->backward_ch = backward_ch[1];
	t[1]->forward_ch = forward_ch[1];
	t[2]->backward_ch = backward_ch[2];
	t[2]->forward_ch = forward_ch[2];
	t[3]->backward_ch = backward_ch[3];
	t[3]->forward_ch = forward_ch[3];
	pwm__config(forward_ch[0]);
	pwm__config(backward_ch[0]);
	pwm__config(forward_ch[1]);
	pwm__config(backward_ch[1]);
	pwm__config(forward_ch[2]);
	pwm__config(backward_ch[2]);
	pwm__config(forward_ch[3]);
	pwm__config(backward_ch[3]);
	/**EMIOS9用于驱动板的pwm3通道输出持续高电平**/
//	pwm__config(EMIOS_CH9);
//	pwm__duty_update(EMIOS_CH9,1.0f);
}

void motor_output(Motor t , float duty)
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
//void Motor__config(struct motor motor,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch)
//{
//	motor.backward_ch = backward_ch;
//	motor.forward_ch = forward_ch;
//	pwm__config(forward_ch);
//	pwm__config(backward_ch);
//	/**EMIOS9用于驱动板的pwm3通道输出持续高电平**/
//	pwm__config(EMIOS_CH9);
//	pwm__duty_update(EMIOS_CH9,1.0f);
//	/***************************************/
////	PID__config(&(motor->motor_pid), kp, ki, kd, period_ms, perror_max, ierror_max, derror_max);
//}

