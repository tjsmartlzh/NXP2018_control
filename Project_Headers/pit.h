/*
 * pit.h
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#ifndef PIT_H_
#define PIT_H_

#include"MPC5604B.h"
#include"IntcInterrupts.h"
typedef enum
{
	PIT_Timer0 = 0,PIT_Timer1,PIT_Timer2,PIT_Timer3,PIT_Timer4,PIT_Timer5,PIT_Timer_LEN = 6
}PIT_Timer;
uint8_t PIT__config(PIT_Timer pit_timer,uint32_t set_time_ms,uint8_t frequency,INTCInterruptFn func, uint8_t pri);

#define IRQ_get(pit_timer) ((pit_timer<=2) ? (59+pit_timer):(124+pit_timer))
#define PIT__clear_flag(pit_timer) (PIT.CH[pit_timer].TFLG.B.TIF = 1)
#define PIT__restart(pit_timer) (PIT.CH[pit_timer].TCTRL.B.TEN = 1)
#define PIT__stop(pit_timer) (PIT.CH[pit_timer].TCTRL.B.TEN = 0)

#endif /* PIT_H_ */

