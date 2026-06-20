/*
 * VescUart.c
 *
 * Ported from the SolidGeek/VescUart Arduino library to plain C
 * for STM32 + STM32CubeIDE HAL (blocking polling UART reads).
 */

#include "vesc/vescuart.h"
#include "vesc/buffer.h"
#include "vesc/crc.h"
#include <string.h>
#include <stdio.h>

/* RX buffer size. Upstream Arduino code used 256, but endMessage can be
 * computed as (length_byte + 5), where length_byte can be up to 255,
 * giving a worst case of 260. Sized to 262 here to avoid an out-of-bounds
 * write in that edge case (this is a small, deliberate fix vs. upstream;
 * normal VESC responses are well under 100 bytes so this rarely matters
 * in practice, but it's free safety margin). */
#define VESC_RX_BUF_SIZE 262

/* ---- internal (private) helpers, not exposed in VescUart.h ---- */
static int  packSendPayload(VescUart_t *vesc, uint8_t *payload, int lenPay);
static int  receiveUartMessage(VescUart_t *vesc, uint8_t *payloadReceived);
static bool unpackPayload(VescUart_t *vesc, uint8_t *message, int lenMes, uint8_t *payload);
static bool processReadPacket(VescUart_t *vesc, uint8_t *message);
static void debugPrint(VescUart_t *vesc, const char *str);
static void debugPrintBytes(VescUart_t *vesc, uint8_t *data, int len);

/* ===================== init / config ===================== */

void VescUart_Init(VescUart_t *vesc, UART_HandleTypeDef *huart, uint32_t timeout_ms) {
	memset(vesc, 0, sizeof(VescUart_t));

	vesc->huart = huart;
	vesc->debug_huart = NULL;
	vesc->timeout_ms = (timeout_ms == 0) ? 100 : timeout_ms;

	vesc->nunchuck.valueX = 127;
	vesc->nunchuck.valueY = 127;
	vesc->nunchuck.lowerButton = false;
	vesc->nunchuck.upperButton = false;
}

void VescUart_SetDebugPort(VescUart_t *vesc, UART_HandleTypeDef *debug_huart) {
	vesc->debug_huart = debug_huart;
}

/* ===================== debug helpers ===================== */

static void debugPrint(VescUart_t *vesc, const char *str) {
	if (vesc->debug_huart != NULL) {
		HAL_UART_Transmit(vesc->debug_huart, (uint8_t *)str, (uint16_t)strlen(str), 100);
	}
}

static void debugPrintBytes(VescUart_t *vesc, uint8_t *data, int len) {
	if (vesc->debug_huart == NULL) {
		return;
	}
	char buf[8];
	for (int i = 0; i <= len; i++) {
		int n = snprintf(buf, sizeof(buf), "%u ", data[i]);
		HAL_UART_Transmit(vesc->debug_huart, (uint8_t *)buf, (uint16_t)n, 100);
	}
	debugPrint(vesc, "\r\n");
}

/* ===================== low-level UART framing ===================== */

static int receiveUartMessage(VescUart_t *vesc, uint8_t *payloadReceived) {

	/* Messages <= 255 start with "2", 2nd byte is length.
	 * Messages > 255 start with "3", not handled here (matches upstream). */

	if (vesc->huart == NULL) {
		return -1;
	}

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t *messageReceived = vesc->_rxBuf;
	uint16_t lenPayload = 0;

	uint32_t startTick = HAL_GetTick();

	while (!messageRead) {

		uint32_t elapsed = HAL_GetTick() - startTick;
		if (elapsed >= vesc->timeout_ms) {
			break;
		}
		uint32_t remaining = vesc->timeout_ms - elapsed;

		/* Blocking wait for exactly one byte, up to 'remaining' ms.
		 * This plays the role of the Arduino available()/read() polling loop. */
		HAL_StatusTypeDef status = HAL_UART_Receive(vesc->huart, &messageReceived[counter], 1, remaining);
		if (status != HAL_OK) {
			break; /* timeout or UART error while waiting for the next byte */
		}
		counter++;

		if (counter == 2) {
			switch (messageReceived[0]) {
				case 2:
					endMessage = messageReceived[1] + 5; /* len + start + len_byte + 2 CRC + end */
					lenPayload = messageReceived[1];
					break;

				case 3:
					debugPrint(vesc, "Message is larger than 256 bytes - not supported\r\n");
					break;

				default:
					debugPrint(vesc, "Invalid start bit\r\n");
					break;
			}
		}

		if (counter >= VESC_RX_BUF_SIZE) {
			break;
		}

		if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
			messageReceived[endMessage] = 0;
			debugPrint(vesc, "End of message reached!\r\n");
			messageRead = true;
			break;
		}
	}

	if (!messageRead) {
		debugPrint(vesc, "Timeout\r\n");
	}

	bool unpacked = false;
	if (messageRead) {
		unpacked = unpackPayload(vesc, messageReceived, endMessage, payloadReceived);
	}

	return unpacked ? lenPayload : 0;
}

static bool unpackPayload(VescUart_t *vesc, uint8_t *message, int lenMes, uint8_t *payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	/* Rebuild CRC from the received message */
	crcMessage = (uint16_t)(message[lenMes - 3] << 8);
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if (vesc->debug_huart != NULL) {
		char buf[32];
		snprintf(buf, sizeof(buf), "CRC received: %u\r\n", crcMessage);
		debugPrint(vesc, buf);
	}

	/* Extract payload */
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if (vesc->debug_huart != NULL) {
		char buf[32];
		snprintf(buf, sizeof(buf), "CRC calc: %u\r\n", crcPayload);
		debugPrint(vesc, buf);
	}

	if (crcPayload == crcMessage) {
		if (vesc->debug_huart != NULL) {
			debugPrint(vesc, "Received: ");
			debugPrintBytes(vesc, message, lenMes);

			debugPrint(vesc, "Payload :      ");
			debugPrintBytes(vesc, payload, message[1] - 1);
		}
		return true;
	}

	return false;
}

static int packSendPayload(VescUart_t *vesc, uint8_t *payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[VESC_RX_BUF_SIZE];

	if (lenPay <= 256) {
		messageSend[count++] = 2;
		messageSend[count++] = (uint8_t)lenPay;
	} else {
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;

	if (vesc->debug_huart != NULL) {
		debugPrint(vesc, "Package to send: ");
		debugPrintBytes(vesc, messageSend, count);
	}

	if (vesc->huart != NULL) {
		HAL_UART_Transmit(vesc->huart, messageSend, (uint16_t)count, 100);
	}

	return count;
}

static bool processReadPacket(VescUart_t *vesc, uint8_t *message) {

	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; /* Drop the packetId byte, rest is payload */

	switch (packetId) {

		case COMM_FW_VERSION:
			vesc->fw_version.major = message[index++];
			vesc->fw_version.minor = message[index++];
			return true;

		case COMM_GET_VALUES:
			vesc->data.tempMosfet       = buffer_get_float16(message, 10.0, &index);
			vesc->data.tempMotor        = buffer_get_float16(message, 10.0, &index);
			vesc->data.avgMotorCurrent  = buffer_get_float32(message, 100.0, &index);
			vesc->data.avgInputCurrent  = buffer_get_float32(message, 100.0, &index);
			index += 4; /* skip avg Id */
			index += 4; /* skip avg Iq */
			vesc->data.dutyCycleNow     = buffer_get_float16(message, 1000.0, &index);
			vesc->data.rpm              = buffer_get_float32(message, 1.0, &index);
			vesc->data.inpVoltage       = buffer_get_float16(message, 10.0, &index);
			vesc->data.ampHours         = buffer_get_float32(message, 10000.0, &index);
			vesc->data.ampHoursCharged  = buffer_get_float32(message, 10000.0, &index);
			vesc->data.wattHours        = buffer_get_float32(message, 10000.0, &index);
			vesc->data.wattHoursCharged = buffer_get_float32(message, 10000.0, &index);
			vesc->data.tachometer       = buffer_get_int32(message, &index);
			vesc->data.tachometerAbs    = buffer_get_int32(message, &index);
			vesc->data.error            = (mc_fault_code)message[index++];
			vesc->data.pidPos           = buffer_get_float32(message, 1000000.0, &index);
			vesc->data.id               = message[index++];
			return true;

		default:
			return false;
	}
}

/* ===================== public API ===================== */

bool VescUart_GetFWVersion(VescUart_t *vesc) {
	return VescUart_GetFWVersionCan(vesc, 0);
}

bool VescUart_GetFWVersionCan(VescUart_t *vesc, uint8_t canId) {

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];

	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_FW_VERSION;

	packSendPayload(vesc, payload, payloadSize);

	uint8_t message[VESC_RX_BUF_SIZE];
	int messageLength = receiveUartMessage(vesc, message);
	if (messageLength > 0) {
		return processReadPacket(vesc, message);
	}
	return false;
}

bool VescUart_GetVescValues(VescUart_t *vesc) {
	return VescUart_GetVescValuesCan(vesc, 0);
}

bool VescUart_GetVescValuesCan(VescUart_t *vesc, uint8_t canId) {

	if (vesc->debug_huart != NULL) {
		char buf[40];
		snprintf(buf, sizeof(buf), "Command: COMM_GET_VALUES %u\r\n", canId);
		debugPrint(vesc, buf);
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_GET_VALUES;

	packSendPayload(vesc, payload, payloadSize);

	uint8_t message[VESC_RX_BUF_SIZE];
	int messageLength = receiveUartMessage(vesc, message);

	if (messageLength > 55) {
		return processReadPacket(vesc, message);
	}
	return false;
}

void VescUart_SetNunchuckValues(VescUart_t *vesc) {
	VescUart_SetNunchuckValuesCan(vesc, 0);
}

void VescUart_SetNunchuckValuesCan(VescUart_t *vesc, uint8_t canId) {

	if (vesc->debug_huart != NULL) {
		char buf[40];
		snprintf(buf, sizeof(buf), "Command: COMM_SET_CHUCK_DATA %u\r\n", canId);
		debugPrint(vesc, buf);
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 11 : 13);
	uint8_t payload[payloadSize];

	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_SET_CHUCK_DATA;
	payload[index++] = (uint8_t)vesc->nunchuck.valueX;
	payload[index++] = (uint8_t)vesc->nunchuck.valueY;
	buffer_append_bool(payload, vesc->nunchuck.lowerButton, &index);
	buffer_append_bool(payload, vesc->nunchuck.upperButton, &index);

	/* Acceleration data, unused: Int16 x3 (6 bytes) */
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;

	if (vesc->debug_huart != NULL) {
		char buf[64];
		snprintf(buf, sizeof(buf), "Nunchuck x=%d y=%d LBTN=%d UBTN=%d\r\n",
				 vesc->nunchuck.valueX, vesc->nunchuck.valueY,
				 vesc->nunchuck.lowerButton, vesc->nunchuck.upperButton);
		debugPrint(vesc, buf);
	}

	packSendPayload(vesc, payload, payloadSize);
}

void VescUart_SetCurrent(VescUart_t *vesc, float current) {
	VescUart_SetCurrentCan(vesc, current, 0);
}

void VescUart_SetCurrentCan(VescUart_t *vesc, float current, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(vesc, payload, payloadSize);
}

void VescUart_SetBrakeCurrent(VescUart_t *vesc, float brakeCurrent) {
	VescUart_SetBrakeCurrentCan(vesc, brakeCurrent, 0);
}

void VescUart_SetBrakeCurrentCan(VescUart_t *vesc, float brakeCurrent, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	packSendPayload(vesc, payload, payloadSize);
}

void VescUart_SetRPM(VescUart_t *vesc, float rpm) {
	VescUart_SetRPMCan(vesc, rpm, 0);
}

void VescUart_SetRPMCan(VescUart_t *vesc, float rpm, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_SET_RPM;
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(vesc, payload, payloadSize);
}

void VescUart_SetDuty(VescUart_t *vesc, float duty) {
	VescUart_SetDutyCan(vesc, duty, 0);
}

void VescUart_SetDutyCan(VescUart_t *vesc, float duty, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);
	packSendPayload(vesc, payload, payloadSize);
}

void VescUart_SendKeepalive(VescUart_t *vesc) {
	VescUart_SendKeepaliveCan(vesc, 0);
}

void VescUart_SendKeepaliveCan(VescUart_t *vesc, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = COMM_FORWARD_CAN;
		payload[index++] = canId;
	}
	payload[index++] = COMM_ALIVE;
	packSendPayload(vesc, payload, payloadSize);
}

void VescUart_PrintVescValues(VescUart_t *vesc) {
	if (vesc->debug_huart == NULL) {
		return;
	}
	char buf[64];

	snprintf(buf, sizeof(buf), "avgMotorCurrent: %.3f\r\n", vesc->data.avgMotorCurrent);       debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "avgInputCurrent: %.3f\r\n", vesc->data.avgInputCurrent);       debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "dutyCycleNow: %.3f\r\n", vesc->data.dutyCycleNow);             debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "rpm: %.1f\r\n", vesc->data.rpm);                               debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "inputVoltage: %.2f\r\n", vesc->data.inpVoltage);               debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "ampHours: %.4f\r\n", vesc->data.ampHours);                     debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "ampHoursCharged: %.4f\r\n", vesc->data.ampHoursCharged);       debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "wattHours: %.4f\r\n", vesc->data.wattHours);                   debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "wattHoursCharged: %.4f\r\n", vesc->data.wattHoursCharged);     debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "tachometer: %ld\r\n", (long)vesc->data.tachometer);             debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "tachometerAbs: %ld\r\n", (long)vesc->data.tachometerAbs);       debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "tempMosfet: %.2f\r\n", vesc->data.tempMosfet);                 debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "tempMotor: %.2f\r\n", vesc->data.tempMotor);                   debugPrint(vesc, buf);
	snprintf(buf, sizeof(buf), "error: %d\r\n", (int)vesc->data.error);                        debugPrint(vesc, buf);
}