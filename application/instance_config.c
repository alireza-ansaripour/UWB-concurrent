/*
 * instance_config.c
 *
 *  Created on: Oct 14, 2019
 *      Author: milad
 */

#include "instance_config.h"
#include "identity.h"
#include "port.h"

dwt_config_t dwt_config = { 7, /* Channel number. */
	DWT_PRF_64M, /* Pulse repetition frequency. */
	DWT_PLEN_2048, /* Preamble length. Used in TX only. */
	DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
	17, /* TX preamble code. Used in TX only. */
	17, /* RX preamble code. Used in RX only. */
	1, /* 0 to use standard SFD, 1 to use non-standard SFD. */
	DWT_BR_110K, /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	(2048 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

dwt_txconfig_t dwt_txconfig = { 0xC9, //PGDelay
		0xE0E0E0E0
//0x80808080
		};

packet_std_t ack_pkt;

void ack_init() {
	uint16 src = identity_get_address();
	uint16 dst = 0;
	ack_pkt.src = src;
	ack_pkt.dst = dst;
	ack_pkt.packet_id = PACKET_ID;
	packet_ack_t *ack = (packet_ack_t*) ack_pkt.payload;
	ack->msg_id = MSG_ACK;
	dwt_writetxdata(offsetof(packet_std_t, payload) + 1, &ack_pkt, 0);
}

void instance_config_identity_init() {
//	switch (STM32_UUID[0]) {
//	case 0x60002E:
//		identity_set_role(ROLE_RX);
		identity_set_address(0x0054);
//		break;
//	case 0x56002F:
//		identity_set_role(ROLE_RX);
//		identity_set_address(0x0001);
//		break;
//	case 0x1B004D:
//		identity_set_role(ROLE_RX);
//		identity_set_address(0x0002);
//		break;
//	case 0x70004C:
//		identity_set_role(ROLE_RX);
//		identity_set_address(0x0003);
//
//	default:
//		identity_append_operations(IDENTITY_OPERATIONS_UUID);
//		identity_set_address(IDENTITY_ADDRESS_BROADCAST);
//		break;
//	}

}

void identity_set_role(Role role) {
	identity_set_operations(IDENTITY_OPERATIONS_NONE);
	instance_info.config.enable_64plen_config = 0;
	switch (role) {
	case ROLE_TX:
		identity_append_operations(IDENTITY_OPERATIONS_DATA_TX);
		if (CIR_LEN)
			identity_append_operations(IDENTITY_OPERATIONS_PERIODIC_TX);
		identity_append_operations(IDENTITY_OPERATIONS_PERIODIC_TX);
//		identity_append_operations(IDENTITY_OPERATIONS_ACK_EN);

		break;
	case ROLE_RX:
		identity_append_operations(IDENTITY_OPERATIONS_DATA_RX);
//		identity_append_operations(IDENTITY_OPERATIONS_ACK_EN);
		break;
	case ROLE_TX_CONT:
		identity_append_operations(IDENTITY_OPERATIONS_DATA_TX);
		identity_append_operations(IDENTITY_OPERATIONS_CONSTANT_TX);
		break;

	}
	configure_node();
}

void config_tx() {
	instance_info.config.radio_config = dwt_config;
	instance_info.config.packet_size = 100;
	instance_info.config.tx_number = 200;
	instance_info.config.tx_config = dwt_txconfig;
	instance_info.config.sequence_number = 0;
	instance_info.config.wait_time = EVENT_DATA_TX_INTERVAL;
	instance_info.config.enable_64plen_config = 0;
}

void configure_node() {

	if (identity_get_operations() & IDENTITY_OPERATIONS_DATA_TX) {
		config_tx();
		instance_info.events.data_tx.enabled = 0;
		instance_info.events.data_tx.packet_size = 10;
		dwt_forcetrxoff();
		dwt_rxreset();
	}
	if ((identity_get_operations() & IDENTITY_OPERATIONS_ALL_UWB_RX)) {
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_setrxtimeout(0);
		if ((identity_get_operations() & IDENTITY_OPERATIONS_DATA_RX)) {

			if ((identity_get_operations() & IDENTITY_OPERATIONS_ACK_EN)) {
//				dwt_setrxaftertxdelay(0);
//				dwt_setpanid(PAN_ID);
//				dwt_setaddress16(0x0001);
//				dwt_enableframefilter(DWT_FF_DATA_EN);
//				dwt_enableautoack(10);
//				dwt_setrxautoreenable(1);
//				dwt_setdblrxbuffmode(1);
			} else {
				/* Activate double buffering. */
//				dwt_setrxautoreenable(1);
//				dwt_setdblrxbuffmode(1);
			}
		}
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

	}
}

