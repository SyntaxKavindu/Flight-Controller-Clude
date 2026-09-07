#ifndef STUB_MAIN_H
#define STUB_MAIN_H
#include "stm32f7xx_hal.h"
extern GPIO_TypeDef *const SPI1_CS_GPIO_Port;
extern GPIO_TypeDef *const SPI2_CS_GPIO_Port;
extern GPIO_TypeDef *const SPI3_CS_GPIO_Port;
#define SPI1_CS_Pin 1u
#define SPI2_CS_Pin 2u
#define SPI3_CS_Pin 4u

/* The indicator LEDs. Defined here so the host build takes the same path a
   configured board does -- Globals.cpp only falls back to placeholder pins (and
   its #warning) when these are absent, which is exactly the state a real
   firmware build is in until the pins are set. Distinct bits, because stub.cpp
   records levels per pin. */
extern GPIO_TypeDef *const GPIOE;
#define LED_SYSTEM_GPIO_Port GPIOE
#define LED_SYSTEM_Pin       GPIO_PIN_3
#define LED_ARM_GPIO_Port    GPIOE
#define LED_ARM_Pin          GPIO_PIN_4
#define LED_GPS_GPIO_Port    GPIOE
#define LED_GPS_Pin          GPIO_PIN_5

/* Test hooks. HAL_GetTick() returns g_stub_tick, which starts at 0 -- the same
   value it used to return unconditionally, so every existing suite is
   unaffected until a test moves it. Indicator is entirely clock-driven and
   cannot be tested at all without this. */
#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t g_stub_tick;
/* Last level written to a pin, and how many writes have happened. The write
   count is what proves the class skips a GPIO access it does not need. */
GPIO_PinState stubGpioLevel(uint16_t pin);
uint32_t      stubGpioWriteCount(void);
void          stubGpioReset(void);
#ifdef __cplusplus
}
#endif
#endif
