/*
 * disableWatchdog.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */
#include "MPC5604B.h"
#include "disableWatchdog.h"

void disableWatchdog(void)
{
	SWT.SR.R = 0x0000c520; /* Write keys to clear soft lock bit */
	SWT.SR.R = 0x0000d928;
	SWT.CR.R = 0x8000010A; /* Clear watchdog enable (WEN) */
}
