/**
 * Application entry point.
 */

#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "port.h"
#include "instance.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include "message_format.h"
#include "identity.h"
#include "swing_buffer.h"
#include "operations.h"

#define DATA_SENDER (identity_get_operations() & IDENTITY_OPERATIONS_DATA_TX)
#define PERIODIC_SENDER (DATA_SENDER && (identity_get_operations() & IDENTITY_OPERATIONS_PERIODIC_TX))
#define ACK_SENDER (identity_get_operations() & (IDENTITY_OPERATIONS_DATA_TX | IDENTITY_OPERATIONS_CONSTANT_TX))
HAL_StatusTypeDef uart_stat;
extern seq_numbers[1000];
extern int ind;
uint8 *prev_ptr;
int main(void)
{
	uint8_t event_led_blink = 0;
	uint32_t counter_led_blink = 0;

	uint8_t * buffer = NULL;
	uint8_t r = 0;

	packet_std_t * packet_std;
	packet_uuid_t * packet_uuid;
	packet_ranging_poll_t * packet_ranging_poll;
	packet_ranging_response_t * packet_ranging_response;
	packet_ranging_final_t * packet_ranging_final;
	packet_blink_t * packet_blink;

	uint8_t new_tx_packet = 0;

	/* Start with board specific hardware init. */
	peripherals_init();
	instance_init();
//	led_on(LED_L);
	while (1)
	{
		if (PERIODIC_SENDER && (instance_info.events.data_tx.enabled == 1)){
			if (portGetTickCount() - instance_info.events.data_tx.timeout < UINT32_HALF){
				new_tx_packet = start_data_tx();
			}
		}
		if ((identity_get_operations() & IDENTITY_OPERATIONS_UUID) && (instance_info.events.uuid_tx.enabled == 1)){
			if (portGetTickCount() - instance_info.events.uuid_tx.timeout < UINT32_HALF){
				new_tx_packet = start_uuid_tx();
			}
		}

		if ((identity_get_operations() & IDENTITY_OPERATIONS_RANGING_MASTER) && (instance_info.events.twr_poll_tx.enabled == 1))
		{
			if (portGetTickCount() - instance_info.events.twr_poll_tx.timeout < UINT32_HALF)
			{
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);
				new_tx_packet = start_ranging_tx();
			}
		}

		if ((identity_get_operations() & IDENTITY_OPERATIONS_RANGING_MASTER) && (instance_info.events.twr_final_tx.enabled == 1))
		{
			if (portGetTickCount() - instance_info.events.twr_final_tx.timeout < UINT32_HALF)
			{
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);

				buffer = uwb_tx_swing_buffer.get_write_buffer();
				r = uwb_tx_swing_buffer.get_free_buffers_length();
				if (r < instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count)
				{
					instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count = r;
				}

				if (buffer)
				{
					instance_info.events.twr_final_tx.enabled = 0;

					if (instance_info.ranging.master.valid == 1)
					{
						instance_info.ranging.master.valid = 0;

						packet_std = (packet_std_t *)buffer;
						packet_std->dst = IDENTITY_ADDRESS_BROADCAST;
						packet_ranging_final = (packet_ranging_final_t *)(packet_std->payload);
						packet_ranging_final->msg_id = MSG_TWR_FINAL;
						packet_ranging_final->twr_poll_tx_ts = instance_info.ranging.master.poll_tx_ts_dw;
						for (int i = 0; i < NUM_ANCHORS; ++i)
						{
							if (instance_info.ranging.master.resp_rx_ts_dw[i].valid == 1)
							{
								instance_info.ranging.master.resp_rx_ts_dw[i].valid = 0;

								packet_ranging_final->twr_resp_rx_ts[i].valid = 1;
								packet_ranging_final->twr_resp_rx_ts[i].value = instance_info.ranging.master.resp_rx_ts_dw[i].value;
							}
							else
							{
								packet_ranging_final->twr_resp_rx_ts[i].valid = 0;
							}
						}
						r = uwb_tx_swing_buffer.release_write_buffer(buffer, offsetof(packet_std_t, payload) + sizeof(packet_ranging_final_t));
						if (r == 1)
						{
							instance_info.diagnostics.uwb.tx.swing_buffer_wrong_fingerprint_write_count++;
						}
						else if (r == 2)
						{
							instance_info.diagnostics.uwb.tx.swing_buffer_oversize_write_count++;
						}

						new_tx_packet = 1;
					}
					else
					{
						// ranging.master not valid
						// invalidate everything in ranging.master
						for (int i = 0; i < NUM_ANCHORS; ++i)
						{
							instance_info.ranging.master.resp_rx_ts_dw[i].valid = 0;
						}
					}
				}
				else
				{
					// cannot send packet, the buffer is full
					instance_info.diagnostics.uwb.tx.swing_buffer_full_count++;
				}
			}
		}

		if ((identity_get_operations() & IDENTITY_OPERATIONS_RANGING_SLAVE) && (instance_info.events.twr_resp_tx.enabled == 1))
		{
			if (portGetTickCount() - instance_info.events.twr_resp_tx.timeout < UINT32_HALF)
			{
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);

				buffer = uwb_tx_swing_buffer.get_write_buffer();
				r = uwb_tx_swing_buffer.get_free_buffers_length();
				if (r < instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count)
				{
					instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count = r;
				}

				if (buffer)
				{
					instance_info.events.twr_resp_tx.enabled = 0;

					if (instance_info.ranging.slave.valid == 1)
					{
						instance_info.events.twr_final_expiry.enabled = 1;
						instance_info.events.twr_final_expiry.timeout = instance_info.ranging.slave.poll_rx_ts_sys + EVENT_TWR_FINAL_EXPIRY_INTERVAL;

						packet_std = (packet_std_t *)buffer;
						packet_std->dst = instance_info.ranging.slave.master_anchor_id;
						packet_ranging_response = (packet_ranging_response_t *)(packet_std->payload);
						packet_ranging_response->msg_id = MSG_TWR_RESP;
						packet_ranging_response->ranging_sequence_number = instance_info.ranging.slave.ranging_sequence_number;
						r = uwb_tx_swing_buffer.release_write_buffer(buffer, offsetof(packet_std_t, payload) + sizeof(packet_ranging_response_t));

						new_tx_packet = 1;
					}
					else
					{
						// ranging.slave not valid
						// invalidate everything in ranging.slave --> nothing more for now

						// release buffer with size of 0
						r = uwb_tx_swing_buffer.release_write_buffer(buffer, 0);
					}
//					instance_info.ranging.slave.resp_tx_ts_dw.valid = 0;

					if (r == 1)
					{
						instance_info.diagnostics.uwb.tx.swing_buffer_wrong_fingerprint_write_count++;
					}
					else if (r == 2)
					{
						instance_info.diagnostics.uwb.tx.swing_buffer_oversize_write_count++;
					}
				}
				else
				{
					// cannot send packet, the buffer is full
					instance_info.diagnostics.uwb.tx.swing_buffer_full_count++;
				}
			}
		}

		if ((identity_get_operations() & IDENTITY_OPERATIONS_BLINK_TX) && (instance_info.events.blink_tx.enabled == 1))
		{
			if (portGetTickCount() - instance_info.events.blink_tx.timeout < UINT32_HALF)
			{
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);
				new_tx_packet = start_blink_tx();
			}
		}

		if ((identity_get_operations() & IDENTITY_OPERATIONS_RANGING_SLAVE) && (instance_info.events.twr_final_expiry.enabled == 1))
		{
			// check twr_final expiration
			if (portGetTickCount() - instance_info.events.twr_final_expiry.timeout < UINT32_HALF)
			{
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);
				gpio_high(DEBUG_MAIN);
				gpio_low(DEBUG_MAIN);

				// expired
				instance_info.events.twr_final_expiry.enabled = 0;
				instance_info.ranging.slave.valid = 0;
				instance_info.ranging.slave.resp_tx_ts_dw.valid = 0;
//				new_tx_packet = 1; // just to trigger interrupt one more time, in case there are packets in tx queue
			}
		}
		instance_process_message();
		// process ring_buffer messages
		uint8_t * data_ptr;
		uint16_t data_len;
		data_ptr = NULL;
		data_len = 0;
		data_len = ring_buffer_get_chunk(&data_ptr);

		if (new_tx_packet){
			instance_tx();
		}
		if (data_len)
		{
			uart_stat = HAL_UART_Transmit_DMA(&USARTx_Handle, data_ptr, data_len);
		}
	}

	return 0;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_DMA(&USARTx_Handle, USARTx_RX_BUFFER, 1);
}
