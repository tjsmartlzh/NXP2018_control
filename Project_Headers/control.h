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
typedef struct motor
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
}motor[4];  //0->nw; 1->ne; 2->sw; 3->se

void vertical(struct motor t[],uint8_t speed_y);
void horizontal(struct motor t[],uint8_t speed_x);
void rotating(struct motor t[],uint8_t angular_speed);
void motor_output(struct motor motor , float duty);
void motor_config(struct motor motor[],EMIOSn_CH forward_ch[],EMIOSn_CH backward_ch[]);

#endif /* ACTION_H_ */
