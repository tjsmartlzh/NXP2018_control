/*
 * clock.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */
#include"clock.h"
void clock__config()
{
	/*初始系统时钟为64MHz*/
	EMIOS_0.MCR.B.GPRE= 63;   //GPRE+1=分频系数；/* Divide 80 MHz sysclk by 79+1 = 80 for 1MHz(0.1us) eMIOS clk*/
	EMIOS_0.MCR.B.GPREN = 1;	/* Enable eMIOS clock */
	EMIOS_0.MCR.B.GTBE = 1;   /* Enable global time base */
	EMIOS_0.MCR.B.FRZ = 1;    /* Enable stopping channels when in debug mode */
}


