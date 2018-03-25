/*
 * pit.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#include"MPC5604B.h"
#include"pit.h"
uint8_t PIT__config(PIT_Timer pit_timer,uint32_t set_time_ms,
		uint8_t frequency ,INTCInterruptFn func, uint8_t pri)
{
	uint8_t IRQ;
	IRQ = IRQ_get(pit_timer);
	if(pit_timer>PIT_Timer5)	return 1;
	
	if(!func || pri < 1 || pri > 14) return 2;
	PIT.PITMCR.B.MDIS = 0;
	PIT.PITMCR.B.FRZ = 1;
	PIT.CH[pit_timer].TCTRL.B.TEN = 0;
	PIT.CH[pit_timer].LDVAL.R = (uint32_t)(set_time_ms *1000*frequency)-1;
	INTC_InstallINTCInterruptHandler(func,IRQ, pri);///<func函数中应清除PIT Timer标志位
	PIT__clear_flag(pit_timer);
	PIT.CH[pit_timer].TCTRL.B.TIE = 1;
	PIT.CH[pit_timer].TCTRL.B.TEN = 1;
	return 0;
}
