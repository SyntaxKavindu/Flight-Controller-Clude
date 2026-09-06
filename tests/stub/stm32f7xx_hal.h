/* Host-side stub of the STM32 HAL, only for compile-checking the flight stack. */
#ifndef STUB_STM32F7XX_HAL_H
#define STUB_STM32F7XX_HAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct { int dummy; } GPIO_TypeDef;
typedef struct { int dummy; } DMA_HandleTypeDef;
typedef struct { uint32_t ErrorCode; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
typedef struct { int dummy; } SPI_HandleTypeDef;
typedef struct { int dummy; } I2C_HandleTypeDef;
typedef struct { uint32_t Instance; uint32_t Init; } TIM_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1u
#define HAL_UART_ERROR_NONE  0u
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 1u
#define TIM_CHANNEL_3 2u
#define TIM_CHANNEL_4 3u

#define __HAL_DMA_GET_COUNTER(h)         (0u)
#define __HAL_TIM_SET_AUTORELOAD(h, v)   ((void)0)
#define __HAL_TIM_SET_PRESCALER(h, v)    ((void)0)
#define __HAL_TIM_SET_COMPARE(h, c, v)   ((void)0)

uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);
void     HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *d, uint16_t s, uint32_t t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t s, uint32_t t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t a, uint32_t tr, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a, uint16_t m, uint16_t ms, uint8_t *d, uint16_t s, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a, uint16_t m, uint16_t ms, uint8_t *d, uint16_t s, uint32_t t);
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *h, uint32_t ch, const uint32_t *d, uint16_t l);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t s);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *h);

#ifdef __cplusplus
}
#endif
#endif
