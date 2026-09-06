/*
 * TelemetryCBridge.h
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * STM32CubeIDE generates usbd_cdc_if.c as plain C, so CDC_Receive_FS() cannot
 * call a C++ method. Include this from usbd_cdc_if.c and forward the received
 * bytes; they are handed to whichever Telemetry instance called init() last.
 *
 *     static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
 *     {
 *         Telemetry_Receive(Buf, *Len);
 *         USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
 *         USBD_CDC_ReceivePacket(&hUsbDeviceFS);
 *         return (USBD_OK);
 *     }
 *
 * Forwards to the global `telemetry`, so there is nothing to bind first.
 *
 * !! Runs in interrupt context, and command handling happens inline: this
 * !! mutates calibrator state the main loop also reads, and transmits from
 * !! inside the USB RX interrupt.
 */

#ifndef TELEMETRY_TELEMETRYCBRIDGE_H_
#define TELEMETRY_TELEMETRYCBRIDGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Telemetry_Receive(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_TELEMETRYCBRIDGE_H_ */
