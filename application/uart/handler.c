/*
 * handler.c
 *
 *  Created on: Oct 14, 2020
 *      Author: 18324
 */
#include "instance.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "port.h"
#include "stm32l4xx_hal.h"
#include "swing_buffer.h"
#include "message_format.h"
#include "radio/util.h"
#include "deca_device_ex.h"
#include "identity.h"
enum Msg_type{
	DATA_OK,
	DATA_ERR,
	CONFIG,
	CONFIG_TX,
	ROLE,
	START_TX,
	PACKET_SIZE,
	SEQ_NUM,
	TX_NUM,
	DUMMY,
	WAIT_TIME,
	ID,
	ACK_EN,
	PLEN64_EN,
};
enum State_machine {
	SYNC_1, SYNC_2, P_LEN, RECV_DATA, DATA_DONE
};
enum State_machine state = SYNC_1;
uint8 packet_len = 0;
uint8 config_buffer[1000];
uint16 data_index = 0;
uint8 UART_HEADER[] = {0x5c, 0x51};
state_machine_state_t state_machine_state;
uint8 UART_ACK[] = {0x5c, 0x51, 2, 0, MSG_CONF, 0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	/* Set transmission flag: trasfer complete*/
	uint8 byte = USARTx_RX_BUFFER[0];
	uint32* seq_num = NULL;
	uint16* wait_time = NULL;
	uint16* addr = NULL;
	uint16* packet_size = NULL;
	uint16* tx_num = NULL;
	dwt_txconfig_t *tx_conf = NULL;
	switch (state){
		case SYNC_1:
			if (byte == UART_HEADER[0]){
				state = SYNC_2;
			}
			break;
		case SYNC_2:
			if (byte == UART_HEADER[1])
				state = P_LEN;
			else
				state = SYNC_1;
			break;
		case P_LEN:
				packet_len = byte;
				data_index = 0;
				state = RECV_DATA;
//				memset(config_buffer, 0, sizeof(config_buffer));
			break;
		case RECV_DATA:
			config_buffer[data_index] = byte;
			data_index++;
			if (data_index == packet_len){
				state = SYNC_1;
				uint8 msg_type = config_buffer[0];
				UART_ACK[sizeof(UART_ACK) - 1] = msg_type;

				switch(msg_type){
					case CONFIG:
						process_config(config_buffer, packet_len);
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;
					case ROLE:
						set_role(config_buffer, packet_len);
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;
					case START_TX:
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						instance_info.events.data_tx.enabled = 1;
						start_data_tx();
						instance_tx();
						break;

					case SEQ_NUM:
						seq_num = (uint32 *)&config_buffer[1];
						instance_info.config.sequence_number = *seq_num;
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;

					case CONFIG_TX:
						tx_conf = (dwt_txconfig_t *)&config_buffer[1];
						dwt_txconfig.PGdly = tx_conf->PGdly;
						dwt_txconfig.power = tx_conf->power;
						dwt_forcetrxoff();
						dwt_configuretxrf(&dwt_txconfig);
						instance_info.config.tx_config = dwt_txconfig;
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;

					case PACKET_SIZE:
						packet_size = (uint16*)&config_buffer[1];
						instance_info.config.packet_size = *packet_size;
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;

					case TX_NUM:
						tx_num = (uint16*)&config_buffer[1];
						instance_info.config.tx_number = *tx_num;
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;

					case WAIT_TIME:
						wait_time = (uint16 *)&config_buffer[1];
						instance_info.config.wait_time = *wait_time;
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;

					case ID:
						addr = (uint16 *)&config_buffer[1];
						identity_set_address(*addr);
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;

					case ACK_EN:
						identity_append_operations(IDENTITY_OPERATIONS_ACK_EN);
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;
					case PLEN64_EN:
						instance_info.config.enable_64plen_config = 1;
						HAL_UART_Transmit_DMA(&USARTx_Handle, UART_ACK, sizeof(UART_ACK));
						break;
				}


			}
			else if (data_index > packet_len)
			{
				state = SYNC_1;
			}
			break;
		default:
			state = SYNC_1;
			break;
	}

	HAL_UART_Receive_DMA(&USARTx_Handle, USARTx_RX_BUFFER, 1);
}



