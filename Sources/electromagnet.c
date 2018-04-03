/*
 * electromagnet.c
 *
 *  Created on: Apr 1, 2018
 *      Author: dell
 */

#include "electromagnet.h"
#include "gpio.h"

void open_elecm()
{
	GPIO__output__enable(11);
	GPIO__output_high(11);
}

void close_elecm()
{
	GPIO__output__enable(11);
	GPIO__output_low(11);
}
