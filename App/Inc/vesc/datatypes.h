/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.
	Trimmed down for use with a UART-based VescUart port (STM32 HAL).

	Only the pieces actually needed to talk to VESC over UART are kept:
	  - COMM_PACKET_ID : command IDs for the packets we send/parse
	  - mc_fault_code  : fault code enum (referenced by mc_values.fault_code)
	  - mc_values      : telemetry struct returned by COMM_GET_VALUES

	IMPORTANT: COMM_PACKET_ID values are positional in the original VESC
	firmware (first entry = 0, then +1 each line). To keep protocol
	compatibility while removing unused entries, every kept entry below
	has its original numeric value written explicitly. Do NOT remove the
	"= N" from any line, and do NOT add new entries without explicitly
	assigning the correct number from the full upstream datatypes.h.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------
// Communication commands (subset actually used by VescUart-style comms)
// Values are pinned to match the full upstream datatypes.h exactly.
// ---------------------------------------------------------------------
typedef enum {
	COMM_FW_VERSION         = 0,
	COMM_GET_VALUES         = 4,
	COMM_SET_DUTY           = 5,
	COMM_SET_CURRENT        = 6,
	COMM_SET_CURRENT_BRAKE  = 7,
	COMM_SET_RPM            = 8,
	COMM_SET_POS            = 9,
	COMM_SET_HANDBRAKE      = 10,
	COMM_SET_SERVO_POS      = 12,
	COMM_ALIVE              = 30,
	COMM_FORWARD_CAN        = 34,
	COMM_SET_CHUCK_DATA     = 35,
	COMM_CUSTOM_APP_DATA    = 36,
	COMM_NRF_START_PAIRING  = 37,
} COMM_PACKET_ID;

// ---------------------------------------------------------------------
// Fault codes (kept complete since mc_values.fault_code can legitimately
// report any of these; this enum is small and order must stay intact)
// ---------------------------------------------------------------------
typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR,
	FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
	FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
	FAULT_CODE_MCU_UNDER_VOLTAGE,
	FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
	FAULT_CODE_ENCODER_SPI,
	FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
	FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
	FAULT_CODE_FLASH_CORRUPTION,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
	FAULT_CODE_UNBALANCED_CURRENTS,
	FAULT_CODE_BRK,
	FAULT_CODE_RESOLVER_LOT,
	FAULT_CODE_RESOLVER_DOS,
	FAULT_CODE_RESOLVER_LOS,
	FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
	FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
	FAULT_CODE_ENCODER_NO_MAGNET,
	FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
	FAULT_CODE_PHASE_FILTER,
} mc_fault_code;

// ---------------------------------------------------------------------
// Telemetry returned by COMM_GET_VALUES (kept as-is from upstream)
// ---------------------------------------------------------------------
typedef struct {
	float v_in;
	float temp_mos;
	float temp_mos_1;
	float temp_mos_2;
	float temp_mos_3;
	float temp_motor;
	float current_motor;
	float current_in;
	float id;
	float iq;
	float rpm;
	float duty_now;
	float amp_hours;
	float amp_hours_charged;
	float watt_hours;
	float watt_hours_charged;
	int   tachometer;
	int   tachometer_abs;
	float position;
	mc_fault_code fault_code;
	int   vesc_id;
	float vd;
	float vq;
} mc_values;

#ifdef __cplusplus
}
#endif

#endif /* DATATYPES_H_ */
