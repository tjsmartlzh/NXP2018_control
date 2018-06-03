/*
 * action.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dell
 */

#include "control.h"
#include "pid.h"

//#include "pwm.h" 
//struct motor Motor[4];  //0->nw; 1->ne; 2->sw; 3->se
//extern X_distance;
//extern Y_distance;
void vertical_output(motor t[],uint8_t speed_y) //向前为正，向后为负
{
	float temp;
	int i=0;
	int j=0;
	t[0]->target_speed = speed_y;
	t[1]->target_speed = speed_y;
	t[2]->target_speed = speed_y;
	t[3]->target_speed = speed_y;
	temp=PID__update(&(t[0]->motor_pid), t[0]->target_speed, t[0]->actual_speed);
	temp=PID__update(&(t[1]->motor_pid), t[1]->target_speed, t[1]->actual_speed);
	temp=PID__update(&(t[2]->motor_pid), t[2]->target_speed, t[2]->actual_speed);
	temp=PID__update(&(t[3]->motor_pid), t[3]->target_speed, t[3]->actual_speed);
	t[0]->duty=t[0]->duty+temp;
	t[1]->duty=(t[1]->duty+temp);
	t[2]->duty=(t[2]->duty+temp);
	t[3]->duty=t[3]->duty+temp;
	for(i=0;i<4;i++)
	{
		if(t[i]->duty>=1) t[i]->duty = 1.0f;
		if(t[i]->duty<=-1) t[i]->duty = -1.0f;
		for(j=0;j<150;j++)
		{
			motor_output(t[i],t[i]->duty);
		}
	}
}

void horizontal_output(motor t[],uint8_t speed_x)  //向右为正，向左为负
{
	float temp;
	int i=0;
	int j=0;
	t[0]->target_speed=speed_x;
	t[1]->target_speed=speed_x;
	t[2]->target_speed=speed_x;
	t[3]->target_speed=speed_x;
	temp=PID__update(&(t[0]->motor_pid), t[0]->target_speed, t[0]->actual_speed);
	temp=PID__update(&(t[1]->motor_pid), t[1]->target_speed, t[1]->actual_speed);
	temp=PID__update(&(t[2]->motor_pid), t[2]->target_speed, t[2]->actual_speed);
	temp=PID__update(&(t[3]->motor_pid), t[3]->target_speed, t[3]->actual_speed);
	t[0]->duty=t[0]->duty+temp;
	t[1]->duty=-(t[1]->duty+temp);
	t[2]->duty=-(t[2]->duty+temp);
	t[3]->duty=t[3]->duty+temp;
	for(i=0;i<4;i++)
	{
		if(t[i]->duty>=1) t[i]->duty = 1.0f;
		if(t[i]->duty<=-1) t[i]->duty = -1.0f;
		for(j=0;j<150;j++)
		{
			motor_output(t[i],t[i]->duty);
		}
	}
}

//void rotating_output(motor t[],uint8_t angular_speed)  //顺时针为正，逆时针为负
//{
//	int i=0;
//	t[0]->duty = -angular_speed * (half_track_distance + half_wheelbase);
//	t[1]->duty = angular_speed * (half_track_distance + half_wheelbase);
//	t[2]->duty = -angular_speed * (half_track_distance + half_wheelbase);
//	t[3]->duty = angular_speed * (half_track_distance + half_wheelbase);
//	for(i=0;i<4;i++)
//	{
//		if(t[i]->duty>=1) t[i]->duty = 1.0f;
//		if(t[i]->duty<=-1) t[i]->duty = -1.0f;
//		motor_output(t[i],t[i]->duty);
//	}
//}

void motor_config(motor t,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch,float kp,float ki,float kd,uint32_t period_ms,float perror_max,float ierror_max,float derror_max)
{
	t->backward_ch = backward_ch;
	t->forward_ch = forward_ch;
	pwm__config(forward_ch);
	pwm__config(backward_ch);
	PID__config(&(t->motor_pid), kp, ki, kd, period_ms, perror_max, ierror_max, derror_max);
}

void motor_output(motor t , float duty)
{
	if(duty>=0.09f)
	{
		pwm__duty_update(t->forward_ch,duty);
		pwm__duty_update(t->backward_ch,0);
	}
	else if(duty<=-0.09f)
	{
		pwm__duty_update(t->backward_ch,-duty);
		pwm__duty_update(t->forward_ch,0);
	}
	else
	{
		pwm__duty_update(t->forward_ch,0);
		pwm__duty_update(t->backward_ch,0);
	}
}

void x_control_update(motor Motor[])
{
	float temp,duty_t;
//	PID__config(pid_object_t pid, float kp, float ki, float kd,
//			uint32_t period_ms, float perror_max, float ierror_max,
//			float derror_max);
	//注意：此处应有PID的分段
	PID__config(&(Motor[0]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	PID__config(&(Motor[1]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	PID__config(&(Motor[2]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	PID__config(&(Motor[3]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	duty_t=Motor[4]->x_distance/10;
	if(duty_t>0.5)
	{
		duty_t=0.5;
	}
	else if(duty_t<-0.5)
	{
		duty_t=-0.5;
	}
	else
	{
		duty_t=duty_t;
	}
	horizontal_output(Motor,duty_t);
//	Motor[0]->target_speed=duty_t;
//	temp=PID__update(Motor[0]->motor_pid, Motor[0]->target_speed, Motor[0]->actual_speed);
//	Motor[0]->duty=Motor[0]->duty+temp;
//	motor_output(Motor[0],Motor[0]->duty);
}

void y_control_update(motor Motor[])
{
	float temp,duty_t;
	PID__config(&(Motor[0]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	PID__config(&(Motor[1]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	PID__config(&(Motor[2]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	PID__config(&(Motor[3]->motor_pid), 0.1,0.1, 0.1,10,10,10,10);
	duty_t=Motor[4]->y_distance/10;
	if(duty_t>0.5)
	{
		duty_t=0.5;
	}
	else if(duty_t<-0.5)
	{
		duty_t=-0.5;
	}
	else
	{
		duty_t=duty_t;
	}
	vertical_output(Motor,duty_t);
//	Motor[0]->target_speed=duty_t;
//	temp=pid_update(Motor[0]->target_speed,Motor[0]->actual_speed,0.1,0.1,0.1);
//	Motor[0]->duty=Motor[0]->duty+temp;
//	motor_output(Motor[0],Motor[0]->duty);
}
