/*
 * pid.h
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#ifndef PID_H_
#define PID_H_

#include"MPC5604B.h"
#include"math.h"
typedef struct {
	float Kp, perror, perror_max;
	float Ki, ierror, ierror_max;
	float Kd, derror, derror_max;
	float period_sec;
	float _output;
} pid_t, *pid_object_t;

/**
 * @param perror_max 单次偏差计算的限值
 * @param ierror_max 积分抗饱和限值
 * @param derror_max 微分幅值限制
 * @param period_ms 调用周期
 */
void PID__config(pid_object_t pid, float kp, float ki, float kd,
		uint32_t period_ms, float perror_max, float ierror_max,
		float derror_max);
/**
 * 在相应的PIT周期中断中调用此函数，以迭代更新控制器中的数据
 * @param actual 外部传感器传入的负反馈值（与目标值同量纲）
 */
float PID__update(pid_object_t pid, float target, float actual);

#define PID__get_output(pid) ((pid)->_output)
#define PID__update_kp(pid, kp) ((pid)->Kp = kp)
#define PID__update_ki(pid, ki) ((pid)->Ki = ki)
#define PID__update_kd(pid, kd) ((pid)->Kd = kd)
#define PID__update_params(pid, kp, ki, kd) ((pid)->Kp = kp, (pid)->Ki = ki, (pid)->Kd = kd)
#define __min__(a,b) ((fabs((a))<(b)) ? (a) : (((a)>0?1.0f:-1.0f)*(b)))
#endif /* PID_H_ */



