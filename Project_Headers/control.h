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
	float target_speed;
	float actual_speed;
	float duty;
	float distance;
	float angel;
	float target_distance;
	float target_angel;
}Motor_t,*motor;

 //0->nw; 1->ne; 2->sw; 3->se
//struct motor Motor[4];  //0->nw; 1->ne; 2->sw; 3->se
void vertical_output(motor t[],uint8_t speed_y);
void horizontal_output(motor t[],uint8_t speed_x);
void rotating_output(motor t[],uint8_t angular_speed);
void motor_output(motor t , float duty);
void motor_config(motor t,EMIOSn_CH forward_ch,EMIOSn_CH backward_ch);
#endif /* ACTION_H_ */
