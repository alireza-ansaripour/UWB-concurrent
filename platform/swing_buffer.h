/*
 * swing_buffer.h
 *
 *  Created on: Jan 18, 2018
 *      Author: milad
 */

#ifndef SRC_PLATFORM_SWING_BUFFER_H_
#define SRC_PLATFORM_SWING_BUFFER_H_

#include <stdint.h>
#include "port.h"

//#define SINGLE_BUFFER_SIZE 4300
//#define SWING_BUFFER_SIZE 4
#define SINGLE_BUFFER_FINGERPRINT 0x00C0FFEE
#define SWING_BUFFER_FINGERPRINT 0xC001CAFE

typedef enum { READY_READ, READY_WRITE, BUSY_READ, BUSY_WRITE } single_buffer_status;

//typedef struct
//{
//	single_buffer_status status;
//	uint8_t buffer[SINGLE_BUFFER_SIZE];
//	uint16_t len;
//	uint32_t fingerprint;
//} single_buffer_t;
//
//typedef struct
//{
//	uint8_t read_index;
//	uint8_t write_index;
//	single_buffer_t buffers[SWING_BUFFER_SIZE];
//	uint32_t fingerprint;
//} swing_buffer_t;
//
//void swing_buffer_init(void);
//uint8_t * swing_buffer_get_read_buffer(void);
//void swing_buffer_release_read_buffer(uint8_t * buffer);
//uint8_t * swing_buffer_get_write_buffer(void);
//void swing_buffer_release_write_buffer(uint8_t * buffer, uint16_t len);

#define DEFINE_SWING_BUFFER(NAME, COUNT, SIZE) \
\
	typedef struct \
	{ \
		single_buffer_status status; \
		uint8_t buffer[SIZE]; \
		uint16_t len; \
		uint32_t fingerprint; \
	} CONCAT(NAME, _single_buffer_t); \
	\
	typedef struct \
	{ \
		uint8_t read_index; \
		uint8_t write_index; \
		CONCAT(NAME, _single_buffer_t) buffers[COUNT]; \
		uint32_t fingerprint; \
	} CONCAT(NAME, _swing_buffer_t); \
	\
	typedef struct \
	{ \
		void (*init)(void); \
		uint8_t * (*get_read_buffer)(void); \
		uint8_t (*release_read_buffer)(uint8_t *); \
		uint8_t * (*get_write_buffer)(void); \
		uint8_t (*release_write_buffer)(uint8_t *, uint16_t len); \
		uint8_t (*get_free_buffers_length)(void); \
	} CONCAT(NAME, _swing_buffer_functions_t); \
	\
	void CONCAT(NAME, _swing_buffer_init)(void); \
	uint8_t * CONCAT(NAME, _swing_buffer_get_read_buffer)(void); \
	uint8_t CONCAT(NAME, _swing_buffer_release_read_buffer)(uint8_t * buffer); \
	uint8_t * CONCAT(NAME, _swing_buffer_get_write_buffer)(void); \
	uint8_t CONCAT(NAME, _swing_buffer_release_write_buffer)(uint8_t * buffer, uint16_t len); \
	uint8_t CONCAT(NAME, _swing_buffer_get_free_buffers_length)(void); \
	\
	extern CONCAT(NAME, _swing_buffer_functions_t) CONCAT(NAME, _swing_buffer);

#define DECLARE_SWING_BUFFER(NAME, COUNT, SIZE) \
	\
	CONCAT(NAME, _swing_buffer_t) CONCAT(NAME, _swing_buffer_s); \
	\
	CONCAT(NAME, _swing_buffer_functions_t) CONCAT(NAME, _swing_buffer) = { \
		.init = &CONCAT(NAME, _swing_buffer_init), \
		.get_read_buffer = &CONCAT(NAME, _swing_buffer_get_read_buffer), \
		.release_read_buffer = &CONCAT(NAME, _swing_buffer_release_read_buffer), \
		.get_write_buffer = &CONCAT(NAME, _swing_buffer_get_write_buffer), \
		.release_write_buffer = &CONCAT(NAME, _swing_buffer_release_write_buffer), \
		.get_free_buffers_length = &CONCAT(NAME, _swing_buffer_get_free_buffers_length) \
	}; \
	\
	void CONCAT(NAME, _swing_buffer_init)(void) \
	{ \
		CONCAT(NAME, _swing_buffer_s).read_index = 0; \
		CONCAT(NAME, _swing_buffer_s).write_index = 0; \
		for (uint8_t i = 0; i < COUNT; ++i) \
		{ \
			CONCAT(NAME, _swing_buffer_s).buffers[i].status = READY_WRITE; \
			CONCAT(NAME, _swing_buffer_s).buffers[i].len = 0; \
			CONCAT(NAME, _swing_buffer_s).buffers[i].fingerprint = SINGLE_BUFFER_FINGERPRINT; \
		} \
		CONCAT(NAME, _swing_buffer_s).fingerprint = SWING_BUFFER_FINGERPRINT; \
	} \
	\
	uint8_t * CONCAT(NAME, _swing_buffer_get_read_buffer)(void) \
	{ \
		uint8_t * ret = NULL; \
		interrupt_disable(); \
		if (CONCAT(NAME, _swing_buffer_s).buffers[CONCAT(NAME, _swing_buffer_s).read_index].status == READY_READ) \
		{ \
			ret = CONCAT(NAME, _swing_buffer_s).buffers[CONCAT(NAME, _swing_buffer_s).read_index].buffer; \
			CONCAT(NAME, _swing_buffer_s).buffers[CONCAT(NAME, _swing_buffer_s).read_index].status = BUSY_READ; \
			CONCAT(NAME, _swing_buffer_s).read_index = (CONCAT(NAME, _swing_buffer_s).read_index + 1) % COUNT; \
		} \
		interrupt_enable(); \
		return ret; \
	} \
	\
	uint8_t CONCAT(NAME, _swing_buffer_release_read_buffer)(uint8_t * buffer) \
	{ \
		uint8_t ret = 0; \
		CONCAT(NAME, _single_buffer_t) * single_buffer = NULL; \
		interrupt_disable(); \
		single_buffer = (CONCAT(NAME, _single_buffer_t) *)((uint8_t *)buffer - offsetof(CONCAT(NAME, _single_buffer_t), buffer)); \
		if (single_buffer->fingerprint == SINGLE_BUFFER_FINGERPRINT) \
		{ \
			single_buffer->status = READY_WRITE; \
			single_buffer->len = 0; \
		} \
		else \
		{ \
			/* wrong pointer! something is really wrong! */ \
			ret = 1; \
		} \
		interrupt_enable(); \
		return ret; \
	} \
	\
	uint8_t * CONCAT(NAME, _swing_buffer_get_write_buffer)(void) \
	{ \
		uint8_t * ret = NULL; \
		interrupt_disable(); \
		if (CONCAT(NAME, _swing_buffer_s).buffers[CONCAT(NAME, _swing_buffer_s).write_index].status == READY_WRITE) \
		{ \
			ret = CONCAT(NAME, _swing_buffer_s).buffers[CONCAT(NAME, _swing_buffer_s).write_index].buffer; \
			CONCAT(NAME, _swing_buffer_s).buffers[CONCAT(NAME, _swing_buffer_s).write_index].status = BUSY_WRITE; \
			CONCAT(NAME, _swing_buffer_s).write_index = (CONCAT(NAME, _swing_buffer_s).write_index + 1) % COUNT; \
		} \
		interrupt_enable(); \
		return ret; \
	} \
	\
	uint8_t CONCAT(NAME, _swing_buffer_release_write_buffer)(uint8_t * buffer, uint16_t len) \
	{ \
		uint8_t ret = 0; \
		CONCAT(NAME, _single_buffer_t) * single_buffer = NULL; \
		uint8_t index; \
		interrupt_disable(); \
		single_buffer = (CONCAT(NAME, _single_buffer_t) *)((uint8_t *)buffer - offsetof(CONCAT(NAME, _single_buffer_t), buffer)); \
		if (single_buffer->fingerprint == SINGLE_BUFFER_FINGERPRINT) \
		{ \
			index = (uint8_t) (single_buffer - CONCAT(NAME, _swing_buffer_s).buffers); \
			if (single_buffer->status == BUSY_WRITE) \
			{ \
				if (len > 0) \
				{ \
					if (len < SIZE) \
					{ \
						single_buffer->len = len; \
						single_buffer->status = READY_READ; \
					} \
					else \
					{ \
						/* Possible memory corruption. Wrote more than size of single buffer. */ \
						ret = 2; \
					} \
				} \
				else \
				{ \
					single_buffer->len = 0; \
					if (CONCAT(NAME, _swing_buffer_s).write_index == (index + 1) % COUNT) \
					{ \
						CONCAT(NAME, _swing_buffer_s).write_index = index; \
					} \
					single_buffer->status = READY_WRITE; \
				} \
			} \
			else \
			{ \
				/* buffer was not begin written on! */ \
			} \
		} \
		else \
		{ \
			/* wrong pointer! something is really wrong! */ \
			ret = 1; \
		} \
		interrupt_enable(); \
		return ret; \
	} \
	\
	uint8_t CONCAT(NAME, _swing_buffer_get_free_buffers_length)(void) \
	{ \
		uint8_t ret = 0; \
		for (uint8_t i = 0; i < COUNT; ++i) \
		{ \
			if (CONCAT(NAME, _swing_buffer_s).buffers[i].status == READY_WRITE) \
			{ \
				ret++; \
			} \
		} \
		return ret; \
	} \



#define UWB_RX_SWING_BUFFER_NAME uwb_rx
#define UWB_RX_SWING_BUFFER_COUNT 2
//#define UWB_RX_SWING_BUFFER_SIZE 4300
#define UWB_RX_SWING_BUFFER_SIZE 800
DEFINE_SWING_BUFFER(UWB_RX_SWING_BUFFER_NAME, UWB_RX_SWING_BUFFER_COUNT, UWB_RX_SWING_BUFFER_SIZE)

#define UWB_TX_SWING_BUFFER_NAME uwb_tx
#define UWB_TX_SWING_BUFFER_COUNT 2
#define UWB_TX_SWING_BUFFER_SIZE 100
DEFINE_SWING_BUFFER(UWB_TX_SWING_BUFFER_NAME, UWB_TX_SWING_BUFFER_COUNT, UWB_TX_SWING_BUFFER_SIZE)

//#define UARTx_RX_SWING_BUFFER_NAME uartx_rx
//#define UARTx_RX_SWING_BUFFER_COUNT 4
//#define UARTx_RX_SWING_BUFFER_SIZE 100
//DEFINE_SWING_BUFFER(UARTx_RX_SWING_BUFFER_NAME, UARTx_RX_SWING_BUFFER_COUNT, UARTx_RX_SWING_BUFFER_SIZE)

#endif /* SRC_PLATFORM_SWING_BUFFER_H_ */
