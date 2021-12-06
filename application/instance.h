/*
 * instance.h
 *
 *  Created on: Jan 17, 2018
 *      Author: milad
 */

#ifndef SRC_APPLICATION_INSTANCE_H_
#define SRC_APPLICATION_INSTANCE_H_

#define HEADER_SIZE(X, Y) offsetof(X, Y)

#include "deca_device_api.h"
#include <stdint.h>
#include "identity.h"
#include "ring_buffer.h"
#include "message_format.h"
#include "instance_config.h"
typedef enum Role{
	ROLE_TX,
	ROLE_RX,
	ROLE_TX_CONT,
}Role;

typedef enum { STATE_RX, STATE_TX } state_machine_state_t;
typedef struct
{
	uint8_t enabled;
	uint32_t timeout;
} event_t;

typedef struct
{
	uint8_t enabled;
	uint32_t timeout;
	uint16_t packet_size;
} event_data_t;


typedef struct
{
	struct configuration_t{
		uint16 packet_size;
		dwt_config_t radio_config;
		dwt_txconfig_t tx_config;
		uint16 tx_number;
		uint32 sequence_number;
		uint32 wait_time;
		uint8 enable_64plen_config;
	}config;
	struct events_t
	{
		event_t uuid_tx;
		event_t twr_poll_tx;
		event_t twr_resp_tx;
		event_t twr_final_tx;
		event_t twr_final_expiry;
		event_t blink_tx;
		event_data_t data_tx;
	} events;

	struct blink_t
	{
		uint8_t count;
	} blink;

	struct ranging_t
	{
		struct ranging_master_t
		{
			uint8_t valid;
			uint32_t poll_tx_ts_sys;
			uint32_t poll_tx_ts_dw;
			uint64v_t resp_rx_ts_dw[10];
		} master;

		struct ranging_slave_t
		{
			uint8_t valid;
			uint16_t master_anchor_id;
			uint8_t ranging_sequence_number;
			uint32_t poll_rx_ts_sys;
			uint64_t poll_rx_ts_dw;
			uint32v_t resp_tx_ts_dw;
		} slave;
	} ranging;

	struct diagnostics_t
	{
		struct uwb_diag_t
		{
			struct uwb_diag_rx_t
			{
				uint16_t received_count;
				uint16_t packets_kept_count;
				uint16_t to_cb_count;
				uint16_t err_cb_count;
				uint16_t swing_buffer_min_free_count;
				uint16_t swing_buffer_full_count;
				uint16_t swing_buffer_wrong_fingerprint_read_count;
				uint16_t swing_buffer_wrong_fingerprint_write_count;
				uint16_t swing_buffer_oversize_write_count; // memory corruption
			} rx;

			struct uwb_diag_tx_t
			{
				uint16_t sent_count;
				uint16_t failed_count;
				uint16_t conf_cb_count;
				uint16_t swing_buffer_min_free_count;
				uint16_t swing_buffer_full_count;
				uint16_t swing_buffer_wrong_fingerprint_read_count;
				uint16_t swing_buffer_wrong_fingerprint_write_count;
				uint16_t swing_buffer_oversize_write_count; // memory corruption
			} tx;
		} uwb;

		struct usart_diag_t
		{
//			struct usart_diag_rx_t
//			{
//				uint16_t swing_buffer_min_free_count;
//				uint16_t swing_buffer_full_count;
//				uint16_t swing_buffer_wrong_fingerprint_read_count;
//				uint16_t swing_buffer_wrong_fingerprint_write_count;
//				uint16_t swing_buffer_oversize_write_count; // memory corruption
//			} rx;

			struct usart_diag_tx_t
			{
				uint16_t write_count;
				uint16_t ring_buffer_max_length;
				uint16_t ring_buffer_full_count;
			} tx;
		} usart;

		struct twr_diag_t
		{
			uint16_t poll_tx_count;
			uint16_t poll_rx_count;
			uint16_t resp_tx_count;
			uint16_t resp_rx_count;
			uint16_t final_tx_count;
			uint16_t final_rx_count;
			uint16_t report_tx_count;
		} twr;

		struct blink_diag_t
		{
			uint16_t tx_count;
			uint16_t rx_count;
			uint16_t report_tx_count;
		} blink;

	} diagnostics;
} instance_info_t;

extern state_machine_state_t state_machine_state;
extern instance_info_t instance_info;

void instance_init(void);
void instance_process_message(void);
void instance_tx(void);

void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);

#endif /* SRC_APPLICATION_INSTANCE_H_ */
