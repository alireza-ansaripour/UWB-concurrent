/*
 * swing_buffer.c
 *
 *  Created on: Jan 18, 2018
 *      Author: milad
 */

#include "swing_buffer.h"
#include "interrupt.h"

#include <stddef.h>

DECLARE_SWING_BUFFER(UWB_RX_SWING_BUFFER_NAME, UWB_RX_SWING_BUFFER_COUNT, UWB_RX_SWING_BUFFER_SIZE)
DECLARE_SWING_BUFFER(UWB_TX_SWING_BUFFER_NAME, UWB_TX_SWING_BUFFER_COUNT, UWB_TX_SWING_BUFFER_SIZE)
//DECLARE_SWING_BUFFER(UARTx_RX_SWING_BUFFER_NAME, UARTx_RX_SWING_BUFFER_COUNT, UARTx_RX_SWING_BUFFER_SIZE)

//swing_buffer_t swing_buffer_s;
//
//void swing_buffer_init(void)
//{
//	swing_buffer_s.read_index = 0;
//	swing_buffer_s.write_index = 0;
//	for (uint8_t i = 0; i < SWING_BUFFER_SIZE; ++i)
//	{
//		swing_buffer_s.buffers[i].status = READY_WRITE;
//		swing_buffer_s.buffers[i].len = 0;
//		swing_buffer_s.buffers[i].fingerprint = SINGLE_BUFFER_FINGERPRINT;
//	}
//	swing_buffer_s.fingerprint = SWING_BUFFER_FINGERPRINT;
//}
//
//uint8_t * swing_buffer_get_read_buffer(void)
//{
//	uint8_t * ret = NULL;
//
//	interrupt_disable();
//	if (swing_buffer_s.buffers[swing_buffer_s.read_index].status == READY_READ)
//	{
//		ret = swing_buffer_s.buffers[swing_buffer_s.read_index].buffer;
//		swing_buffer_s.buffers[swing_buffer_s.read_index].status = BUSY_READ;
//		swing_buffer_s.read_index = (swing_buffer_s.read_index + 1) % SWING_BUFFER_SIZE;
//	}
//	interrupt_enable();
//	return ret;
//}
//
//void swing_buffer_release_read_buffer(uint8_t * buffer)
//{
//	single_buffer_t * single_buffer = NULL;
//
//	interrupt_disable();
//	single_buffer = (single_buffer_t *)((uint8_t *)buffer - offsetof(single_buffer_t, buffer));
//	if (single_buffer->fingerprint == SINGLE_BUFFER_FINGERPRINT)
//	{
//		single_buffer->status = READY_WRITE;
//		single_buffer->len = 0;
//	}
//	interrupt_enable();
//}
//
//uint8_t * swing_buffer_get_write_buffer(void)
//{
//	uint8_t * ret = NULL;
//
//	interrupt_disable();
//	if (swing_buffer_s.buffers[swing_buffer_s.write_index].status == READY_WRITE)
//	{
//		ret = swing_buffer_s.buffers[swing_buffer_s.write_index].buffer;
//		swing_buffer_s.buffers[swing_buffer_s.write_index].status = BUSY_WRITE;
//		swing_buffer_s.write_index = (swing_buffer_s.write_index + 1) % SWING_BUFFER_SIZE;
//	}
//	interrupt_enable();
//	return ret;
//}
//
//void swing_buffer_release_write_buffer(uint8_t * buffer, uint16_t len)
//{
//	single_buffer_t * single_buffer = NULL;
//	uint8_t index;
//
//	interrupt_disable();
//	single_buffer = (single_buffer_t *)((uint8_t *)buffer - offsetof(single_buffer_t, buffer));
//	if (single_buffer->fingerprint == SINGLE_BUFFER_FINGERPRINT)
//	{
//		index = (uint8_t) (single_buffer - swing_buffer_s.buffers);
//		if (single_buffer->status == BUSY_WRITE)
//		{
//			if (len > 0)
//			{
//				if (len < SINGLE_BUFFER_SIZE)
//				{
//					single_buffer->len = len;
//					single_buffer->status = READY_READ;
//				}
//				else
//				{
//					// Possible memory corruption. Wrote more than size of single buffer.
//				}
//			}
//			else
//			{
//				single_buffer->len = 0;
//				if (swing_buffer_s.write_index == (index + 1) % SWING_BUFFER_SIZE)
//				{
//					swing_buffer_s.write_index = index;
//				}
//				single_buffer->status = READY_WRITE;
//			}
//		}
//		else
//		{
//			// buffer was not begin written on!
//		}
//	}
//	else
//	{
//		// wrong pointer! something is really wrong!
//	}
//	interrupt_enable();
//}
