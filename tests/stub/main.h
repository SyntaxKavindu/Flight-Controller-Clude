#ifndef STUB_MAIN_H
#define STUB_MAIN_H
#include "stm32f7xx_hal.h"
extern GPIO_TypeDef *const SPI1_CS_GPIO_Port;
extern GPIO_TypeDef *const SPI2_CS_GPIO_Port;
extern GPIO_TypeDef *const SPI3_CS_GPIO_Port;
#define SPI1_CS_Pin 1u
#define SPI2_CS_Pin 2u
#define SPI3_CS_Pin 4u
#endif
