/*
 * ledStatus.cpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Itamar Eliakim
 */

#include "stm32f4xx_hal.h"
#include "nbt.h"

static nbt_t lednbt;

extern "C" void init_ledStatus()
{
	NBT_init(&lednbt, 1000);
}

extern "C" void ledStatus_handler()
{
	if (NBT_handler(&lednbt))
	{

	}
}
