/*
 * interrupt.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#include "interrupt.h"
#include "MPC5604B.h"
void enableIrq(void) 
{
	  INTC.CPR.B.PRI = 0;          /* Single Core: Lower INTC's current priority */
	  asm(" wrteei 1");	    	   /* Enable external interrupts */
}
