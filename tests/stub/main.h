#ifndef STUB_MAIN_H
#define STUB_MAIN_H
#include "stm32f7xx_hal.h"
/* Mirrors the board's generated main.h, so the host build sees the same labels,
   ports and pin bits the firmware does. Kept in step by hand -- if a pin moves
   in CubeMX, move it here too, or a mismatch will only show up on hardware. */
extern GPIO_TypeDef *const GPIOA;
extern GPIO_TypeDef *const GPIOB;
extern GPIO_TypeDef *const GPIOC;
extern GPIO_TypeDef *const GPIOD;

#define SYSTEM_Pin        GPIO_PIN_13
#define SYSTEM_GPIO_Port  GPIOC
#define GPS_Pin           GPIO_PIN_14
#define GPS_GPIO_Port     GPIOC
#define ARM_Pin           GPIO_PIN_15
#define ARM_GPIO_Port     GPIOC
#define SPI1_CS_Pin       GPIO_PIN_2
#define SPI1_CS_GPIO_Port GPIOD
#define SPI2_CS_Pin       GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI3_CS_Pin       GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA

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
