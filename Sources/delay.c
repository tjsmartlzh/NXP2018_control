/*
 * delay.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#include "delay.h"
void delay_us(int us)
{
	int i;
	//系统主频为64MHz
	for(i = 0;i<us;i++)
	{
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);//__asm(nop)为一个机器周期时间
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
		__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);__asm(nop);
	}
}
void delay_ms(int ms)
{
	int i;
	for(i = 0;i<ms;i++)delay_us(1000);
}
void delay_s(int s)
{
	int i;
	for( i = 0;i<s;i++)delay_ms(s);
}
