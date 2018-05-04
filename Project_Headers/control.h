/*
 * action.h
 *
 *  Created on: Mar 26, 2018
 *      Author: dell
 */

#ifndef ACTION_H_
#define ACTION_H_

#include "MPC5604B.h"
#include "pid.h"
#include "pwm.h"
#define half_track_distance 10 //°ëÂÖ¾à
#define half_wheelbase 10  //°ëÖá¾à
typedef struct
{
	EMIOSn_CH forward_ch;
    EMIOSn_CH backward_ch;
    pid_t motor_pid;
	float target_speed;
	float actual_speed;
	float duty;
	float x_distance;
	float angel;
	float y_distance;
	float target_angel;
}Motor_t,*motor;

 //0->nw; 1->ne; 2->sw; 3->se
//struct motor Motor[4];  //0->nw; 1->ne; 2->sw; 3->se
void vertical_output(motor t[],uint8_t speed_y);
void horizontal_output(motor t[],uint8_t speed_x);
void rotating_output(motor t[],uint8_t angular_speed);
void motor_output(motor t , float duty);
void motor_config(motor t,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch,float kp,float ki,float kd,uint32_t period_ms,float perror_max,float ierror_max,float derror_max);
void x_control_update(motor Motor[]);
void y_control_update(motor Motor[]);
#endif /* ACTION_H_ */
