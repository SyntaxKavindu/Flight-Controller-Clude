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

// Distinct objects so a port mix-up is at least visible in a debugger, though
// HAL_GPIO_WritePin() here records by pin bit alone -- see the note there.
static GPIO_TypeDef g_gpioa, g_gpiob, g_gpioc, g_gpiod;
GPIO_TypeDef *const GPIOA = &g_gpioa;
GPIO_TypeDef *const GPIOB = &g_gpiob;
GPIO_TypeDef *const GPIOC = &g_gpioc;
GPIO_TypeDef *const GPIOD = &g_gpiod;
SPI_HandleTypeDef hspi1, hspi2, hspi3;
I2C_HandleTypeDef hi2c1, hi2c3;

// Telemetry output is swallowed by default. Set to true in a test that wants to
// see what the flight stack would have said on the link.
bool g_stub_echo_telemetry = false;

extern "C" {
uint32_t SystemCoreClock = 216000000u;
uint32_t g_stub_tick = 0u;
uint32_t HAL_GetTick(void) { return g_stub_tick; }
HAL_TickFreqTypeDef HAL_GetTickFreq(void) { return HAL_TICK_FREQ_1KHZ; }
void HAL_Delay(uint32_t) {}
// Recorded rather than swallowed, so a test can assert both WHAT was driven
// and HOW OFTEN. Keyed by pin alone: every stub port is the same object, and
// the pins the tests care about are distinct.
static GPIO_PinState g_pin_level[16];
static uint32_t g_gpio_writes = 0u;
static inline uint8_t pinSlot(uint16_t pin) {
	uint8_t n = 0;
	while (n < 15u && (pin >> n) != 1u) n++;   // GPIO_PIN_n is 1 << n
	return n;
}
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t pin, GPIO_PinState state) {
	g_gpio_writes++;
	g_pin_level[pinSlot(pin)] = state;
}
GPIO_PinState stubGpioLevel(uint16_t pin) { return g_pin_level[pinSlot(pin)]; }
uint32_t stubGpioWriteCount(void) { return g_gpio_writes; }
void stubGpioReset(void) {
	g_gpio_writes = 0u;
	for (unsigned i = 0; i < 16u; i++) g_pin_level[i] = GPIO_PIN_RESET;
}
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
