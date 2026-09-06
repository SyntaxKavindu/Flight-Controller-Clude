/*
 * common.h
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_

// The board-coupled header: the maths types plus the STM32 HAL.
//
// Units that are pure algorithm should include MathTypes.hpp instead -- see the
// note at the top of it. This header is for the drivers and frontends that
// actually talk to a peripheral, and it stays a superset of MathTypes.hpp so
// nothing that includes it needs to change.
#include "MathTypes.hpp"
#include "stm32f7xx_hal.h"

#endif /* COMMON_COMMON_H_ */
