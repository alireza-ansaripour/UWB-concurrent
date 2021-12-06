/*
 * operations.c
 *
 *  Created on: Oct 12, 2020
 *      Author: alireza
 */

#include "swing_buffer.h"
#include "instance.h"
#include "message_format.h"
#include "identity.h"
packet_std_t * packet_std;
packet_uuid_t * packet_uuid;
packet_ranging_poll_t * packet_ranging_poll;
packet_ranging_response_t * packet_ranging_response;
packet_ranging_final_t * packet_ranging_final;
packet_blink_t * packet_blink;
uint8_t * buffer = NULL;
uint8_t r = 0;


uint8 start_data_tx(){
	uint8_t new_tx_packet = 0;
	buffer = uwb_tx_swing_buffer.get_write_buffer();
	if (identity_get_operations() & IDENTITY_OPERATIONS_PERIODIC_TX){
		instance_info.events.data_tx.timeout = portGetTickCount() + instance_info.config.wait_time;
	}
	if (buffer)
	{
		packet_std = (packet_std_t *)buffer;
		packet_std->dst = 0x0001;
		packet_data_t *data_pkt = (packet_data_t *) packet_std->payload;
		data_pkt->msg_id = MSG_DATA;
//		data_pkt->length = MAX_DATA_PAYLOAD_SIZE;
		data_pkt->length = instance_info.config.packet_size;
//		memset(data_pkt->payload, 1, MAX_DATA_PAYLOAD_SIZE);
		r = uwb_tx_swing_buffer.release_write_buffer(buffer, offsetof(packet_std_t, payload) + sizeof(packet_blink_t));
		new_tx_packet = 1;
	}

	return 1;
}

uint8_t start_ranging_tx(){
	uint8_t new_tx_packet = 0;

	buffer = uwb_tx_swing_buffer.get_write_buffer();
	r = uwb_tx_swing_buffer.get_free_buffers_length();
	if (r < instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count)
	{
		instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count = r;
		instance_info.events.blink_tx.timeout += EVENT_DATA_TX_INTERVAL;

	}

	if (buffer)
	{
		instance_info.events.twr_poll_tx.timeout = portGetTickCount() + EVENT_TWR_POLL_TX_INTERVAL;
		instance_info.events.twr_final_tx.enabled = 1;
		instance_info.events.twr_final_tx.timeout = portGetTickCount() + EVENT_TWR_FINAL_TX_INTERVAL;

		packet_std = (packet_std_t *)buffer;
		packet_std->dst = IDENTITY_ADDRESS_BROADCAST;
		packet_ranging_poll = (packet_ranging_poll_t *)(packet_std->payload);
		packet_ranging_poll->msg_id = MSG_TWR_POLL;
		r = uwb_tx_swing_buffer.release_write_buffer(buffer, offsetof(packet_std_t, payload) + sizeof(packet_ranging_poll_t));
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
		// cannot send packet, the buffer is full
		instance_info.diagnostics.uwb.tx.swing_buffer_full_count++;
	}
	return new_tx_packet;

}

uint8 start_blink_tx(){
	uint8_t new_tx_packet = 0;
	buffer = uwb_tx_swing_buffer.get_write_buffer();
	r = uwb_tx_swing_buffer.get_free_buffers_length();
	if (r < instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count)
	{
		instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count = r;
	}

	if (buffer)
	{
		if (instance_info.blink.count++ < EVENT_BLINK_TX_COUNT)
		{
			instance_info.events.blink_tx.timeout += NUM_TAGS * EVENT_BLINK_TX_INTERVAL;

			packet_std = (packet_std_t *)buffer;
			packet_std->dst = IDENTITY_ADDRESS_BROADCAST;
			packet_blink = (packet_blink_t *)(packet_std->payload);
			packet_blink->msg_id = MSG_BLINK;
			// packet_blink->tx_timestamp will be set by instance_tx
			r = uwb_tx_swing_buffer.release_write_buffer(buffer, offsetof(packet_std_t, payload) + sizeof(packet_blink_t));

			new_tx_packet = 1;
		}
		else
		{
//						instance_info.events.blink_tx.enabled = 0;
			instance_info.events.blink_tx.timeout += EVENT_BLINK_TX_AUTO_INCREMENT;
			instance_info.blink.count = 0;
			r = uwb_tx_swing_buffer.release_write_buffer(buffer, 0);
		}

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
	return new_tx_packet;
}

uint8 start_uuid_tx(){
	uint8_t new_tx_packet = 0;
	buffer = uwb_tx_swing_buffer.get_write_buffer();
	r = uwb_tx_swing_buffer.get_free_buffers_length();
	if (r < instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count)
	{
		instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count = r;
	}

	if (buffer)
	{
		instance_info.events.uuid_tx.timeout = portGetTickCount() + EVENT_UUID_TX_INTERVAL;

		packet_std = (packet_std_t *)buffer;
		packet_std->dst = IDENTITY_ADDRESS_BROADCAST;
		packet_uuid = (packet_uuid_t *)(packet_std->payload);
		packet_uuid->msg_id = MSG_UUID;
		packet_uuid->uuid[0] = STM32_UUID[0];
		packet_uuid->uuid[1] = STM32_UUID[1];
		packet_uuid->uuid[2] = STM32_UUID[2];
		r = uwb_tx_swing_buffer.release_write_buffer(buffer, offsetof(packet_std_t, payload) + sizeof(packet_uuid_t));
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
		// cannot send packet, the buffer is full
		instance_info.diagnostics.uwb.tx.swing_buffer_full_count++;
	}
	return new_tx_packet;
}
