/*
 * interrupt.c
 *
 *  Created on: Jan 18, 2018
 *      Author: milad
 */

#include "interrupt.h"
#include <stdint.h>
#include "stm32l433xx.h"

static uint64_t interrupt_counter = 0;

void interrupt_disable(void)
{
//	interrupt_counter++;
//	if (interrupt_counter == 0)
	if (interrupt_counter++ == 0)
	{
		__disable_irq();
	}
}

void interrupt_enable(void)
{
	if (interrupt_counter > 0)
	{
		interrupt_counter--;
		if (interrupt_counter == 0)
		{
			__enable_irq();
		}
	}
}
