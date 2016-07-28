/*
 * cc_gpio.h
 *
 *  Created on: Nov 28, 2013
 *      Author: kreyl
 */

#pragma once

#include "board.h"

#if defined STM32L1XX || defined STM32L4XX
#if CC_GDO0 == 0
#define GDO0_IRQ_HANDLER     Vector58
#elif CC_GDO0 == 1
#define GDO0_IRQ_HANDLER     Vector5C
#elif CC_GDO0 == 2
#define GDO0_IRQ_HANDLER     Vector60
#elif CC_GDO0 == 3
#define GDO0_IRQ_HANDLER     Vector64
#elif CC_GDO0 == 4
#define GDO0_IRQ_HANDLER     Vector68
#endif

#endif
