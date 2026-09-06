/*
 * Host-side stand-ins for the STM32 HAL and the USB CDC endpoint.
 *
 * These exist so the flight stack can be COMPILED AND RUN on a workstation.
 * Nothing here models hardware behaviour -- the peripherals always succeed and
 * the clock does not advance. That is deliberate: these tests exercise the
 * estimator and the calibration maths, which are pure functions of their
 * inputs, and a fake that pretended to be an ICM-42688-P would only test the
 * fake. Timing, SPI behaviour and sensor noise belong on the bench.
 */
#include "main.h"
#include "spi.h"
#include "i2c.h"

static GPIO_TypeDef g_port;
GPIO_TypeDef *const SPI1_CS_GPIO_Port = &g_port;
GPIO_TypeDef *const SPI2_CS_GPIO_Port = &g_port;
GPIO_TypeDef *const SPI3_CS_GPIO_Port = &g_port;
SPI_HandleTypeDef hspi1, hspi2, hspi3;
I2C_HandleTypeDef hi2c1, hi2c3;

// Telemetry output is swallowed by default. Set to true in a test that wants to
// see what the flight stack would have said on the link.
bool g_stub_echo_telemetry = false;

extern "C" {
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t) {}
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState) {}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef*, uint16_t, uint32_t, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef*, uint32_t, const uint32_t*, uint16_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef*) { return HAL_OK; }
uint8_t CDC_Transmit_FS(uint8_t*, uint16_t) { return 0; }
}

// Drone.cpp owns the real one, and the suites that do not link it still pull
// Telemetry in for the calibrator's reporting. Weak so a suite that DOES link
// Drone.cpp overrides it rather than colliding.
__attribute__((weak)) void Drone_ReportDiagnostics(void) {}
