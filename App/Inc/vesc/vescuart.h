/*
 * VescUart.h
 *
 * Ported from the SolidGeek/VescUart Arduino library to plain C
 * for STM32 + STM32CubeIDE HAL, using blocking polling UART reads.
 *
 * Original class -> C struct (VescUart_t) + functions.
 * Function overloads (e.g. setCurrent(current) / setCurrent(current, canId))
 * are split into two uniquely named functions:
 *   VescUart_SetCurrent(vesc, current)            -> canId = 0 (local VESC)
 *   VescUart_SetCurrentCan(vesc, current, canId)  -> forwarded over CAN
 */

#ifndef VESCUART_H_
#define VESCUART_H_

#include <stdint.h>
#include <stdbool.h>

/* Brings in UART_HandleTypeDef, HAL_UART_Receive/Transmit, HAL_GetTick.
 * In a real CubeIDE project this is provided by "main.h" (or "usart.h").
 */
#include "main.h"

#include "datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Telemetry data returned by the VESC (COMM_GET_VALUES) */
typedef struct {
	float avgMotorCurrent;
	float avgInputCurrent;
	float dutyCycleNow;
	float rpm;
	float inpVoltage;
	float ampHours;
	float ampHoursCharged;
	float wattHours;
	float wattHoursCharged;
	int32_t tachometer;
	int32_t tachometerAbs;
	float tempMosfet;
	float tempMotor;
	float pidPos;
	uint8_t id;
	mc_fault_code error;
} VescUart_DataPackage;

/** Nunchuck-style joystick values sent to the VESC */
typedef struct {
	int valueX;
	int valueY;
	bool upperButton;
	bool lowerButton;
} VescUart_NunchuckPackage;

/** Firmware version reported by the VESC */
typedef struct {
	uint8_t major;
	uint8_t minor;
} VescUart_FWVersionPackage;

/** Replaces the VescUart class instance. One of these per VESC/UART. */
typedef struct {
	UART_HandleTypeDef *huart;        /* UART connected to the VESC               */
	UART_HandleTypeDef *debug_huart;  /* Optional debug UART, NULL = debug off    */
	uint32_t timeout_ms;              /* Overall response timeout (ms)            */

	VescUart_DataPackage      data;
	VescUart_NunchuckPackage  nunchuck;
	VescUart_FWVersionPackage fw_version;

	/* Internal scratch buffer used while receiving a frame. Each VescUart_t
	 * instance owns its own buffer so multiple VESCs (multiple UARTs) can be
	 * used safely without sharing state. Treat as private - do not touch
	 * from outside VescUart.c. */
	uint8_t _rxBuf[262];
} VescUart_t;

/**
 * @brief Initialize a VescUart_t instance.
 * @param vesc       Pointer to the instance to initialize
 * @param huart      UART peripheral connected to the VESC
 * @param timeout_ms Response timeout in ms (0 -> defaults to 100)
 */
void VescUart_Init(VescUart_t *vesc, UART_HandleTypeDef *huart, uint32_t timeout_ms);

/**
 * @brief Enable/disable debug printing over a separate UART.
 * @param debug_huart UART to print debug text to, or NULL to disable.
 */
void VescUart_SetDebugPort(VescUart_t *vesc, UART_HandleTypeDef *debug_huart);

/** Populate vesc->fw_version. Returns true on success. */
bool VescUart_GetFWVersion(VescUart_t *vesc);
bool VescUart_GetFWVersionCan(VescUart_t *vesc, uint8_t canId);

/** Populate vesc->data with telemetry. Returns true on success. */
bool VescUart_GetVescValues(VescUart_t *vesc);
bool VescUart_GetVescValuesCan(VescUart_t *vesc, uint8_t canId);

/** Send vesc->nunchuck joystick/button values to the VESC. */
void VescUart_SetNunchuckValues(VescUart_t *vesc);
void VescUart_SetNunchuckValuesCan(VescUart_t *vesc, uint8_t canId);

/** Set motor current in Amps. */
void VescUart_SetCurrent(VescUart_t *vesc, float current);
void VescUart_SetCurrentCan(VescUart_t *vesc, float current, uint8_t canId);

/** Set braking current in Amps. */
void VescUart_SetBrakeCurrent(VescUart_t *vesc, float brakeCurrent);
void VescUart_SetBrakeCurrentCan(VescUart_t *vesc, float brakeCurrent, uint8_t canId);

/** Set RPM (eRPM = mechanical RPM * pole pairs). */
void VescUart_SetRPM(VescUart_t *vesc, float rpm);
void VescUart_SetRPMCan(VescUart_t *vesc, float rpm, uint8_t canId);

/** Set duty cycle, range -1.0 .. 1.0. */
void VescUart_SetDuty(VescUart_t *vesc, float duty);
void VescUart_SetDutyCan(VescUart_t *vesc, float duty, uint8_t canId);

/** Send a keepalive/alive packet (call periodically, e.g. every <1s). */
void VescUart_SendKeepalive(VescUart_t *vesc);
void VescUart_SendKeepaliveCan(VescUart_t *vesc, uint8_t canId);

/** Debug helper: print the contents of vesc->data over the debug UART. */
void VescUart_PrintVescValues(VescUart_t *vesc);

#ifdef __cplusplus
}
#endif

#endif /* VESCUART_H_ */
