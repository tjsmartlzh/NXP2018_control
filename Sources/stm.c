/*
 * stm.c
 *
 *  Created on: Jun 2, 2018
 *      Author: dell
 */
#include "MPC5604B.h"
#include "stm.h"

void STM_init(void)
{
	STM.CR.B.TEN=1;
	STM.CR.B.CPS=0x40; //64
	STM.CR.B.FRZ=1;
}
