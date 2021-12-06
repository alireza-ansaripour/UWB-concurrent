/*
 * instance.c
 *
 *  Created on: Jan 17, 2018
 *      Author: milad
 */

#include "instance.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "port.h"
#include "stm32l4xx_hal.h"
#include "swing_buffer.h"
#include "message_format.h"
#include "deca_device_ex.h"
#include "instance_config.h"
#include "identity.h"
#include "radio/util.h"
typedef enum { PACKET_KEEP, PACKET_KEEP_WITH_CIR, PACKET_DROP } packet_status_t;
instance_info_t instance_info;
//volatile uint32 __attribute__((section(".ram2"))) seq_numbers[2000];

int ind = 0;


int index = 0;
//enum Msg_type stat =
uint32_t sequence_number = 0;
uint8_t ranging_sequence_number = 255;
volatile packet_report_t __attribute__((section(".ram2"))) packet_report;
volatile packet_std_t __attribute__((section(".ram2"))) ack;
uint8 UART_TX_DONE_MSG[] = {0x5C, 0x51, 5, 0, MSG_CONF, 0, 0, 0, 0};

//packet_report_t packet_report;

void instance_init(void)
{
	/* Install DW1000 IRQ handler. */
	port_set_deca_isr(dwt_isr);

	/* Reset and initialize DW1000.
	 * For initialization, DW1000 clocks must be temporarily set to crystal speed. After initialization SPI rate can be increased for optimum
	 * performance. */
	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		while (1)
		{ };
	}
	spi_set_rate_high();

	/* Configure DW1000. See NOTE 6 below. */
//	dwt_configure(&config[CONFIG_INDEX]);
	dwt_configure(&dwt_config);

	/* Register RX call-back. */
	dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

	/* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

	/* Disable response frame timeout. */
	dwt_setrxtimeout(0);


	// disable antenna delay deduction.
	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	// disable smart tx power and set tx power manually
	dwt_setsmarttxpower(0);
	dwt_configuretxrf(&dwt_txconfig);

	/* Initialize uwb rx swing buffer used for pushing packets from interrupt domain to main */
	uwb_rx_swing_buffer.init();

	/* Initialize uwb tx swing buffer used for pushing packets main to interrupt domain */
	uwb_tx_swing_buffer.init();

	/* Initialize uartx rx swing buffer used for receiving uartx packets */
	uwb_tx_swing_buffer.init();

	/* Initialize ring buffer used for transmitting uartx packets */
	ring_buffer_init();

	/* Activate the RNG peripheral */
	RNG_Handle.Instance = RNG;
	HAL_RNG_Init(&RNG_Handle);
	HAL_UART_Receive_DMA(&USARTx_Handle, USARTx_RX_BUFFER,1);
	/* Initialize UUID-based identity */
	memset(&instance_info, 0, sizeof(instance_info_t));
	instance_config_identity_init();

	if (identity_get_operations() && IDENTITY_OPERATIONS_PERIODIC_TX){
		instance_info.events.data_tx.enabled = 1;
		instance_info.events.data_tx.timeout = EVENT_DATA_TX_INTERVAL;
	}
	/* Initialize instance_info */

	instance_info.diagnostics.uwb.rx.swing_buffer_min_free_count = -1;
	instance_info.diagnostics.uwb.tx.swing_buffer_min_free_count = -1;

	gpio_low(DEBUG_RX);
	gpio_low(DEBUG_TX);
	gpio_low(DEBUG_MAIN);

}

void instance_process_message(void)
{
	uint8_t * buffer = NULL;
	uint8_t r = 0;
	packet_info_t * packet_info = NULL;
	packet_info_with_cir_t * packet_info_with_cir = NULL;
	packet_std_t * packet_std = NULL;

	packet_report_data_t * packet_report_data = NULL;
	packet_report_ranging_t * packet_report_ranging = NULL;
	packet_report_blink_t * packet_report_blink = NULL;

	packet_data_t * packet_data = NULL;
	packet_ranging_final_t * packet_ranging_final = NULL;
	additional_ranging_final_t * additional_ranging_final = NULL;
	packet_blink_t * packet_blink = NULL;
	additional_data_t * additional_data = NULL;


	buffer = uwb_rx_swing_buffer.get_read_buffer();

	if (buffer)
	{
		gpio_high(DEBUG_MAIN);
		packet_info = (packet_info_t *)buffer;
		packet_info_with_cir = (packet_info_with_cir_t *)buffer;
		packet_std = (packet_std_t *)packet_info->packet_data;
		packet_report.sync = UART_SYNC;
		packet_report.length = 0;
		switch (packet_std->payload[0])
		{
			case MSG_DATA:
				packet_data = (packet_data_t *)packet_std->payload;
				additional_data = (additional_data_t *)packet_info->additional_data;
				packet_report_data = (packet_report_data_t *)packet_report.data;

				packet_report_data->msg_id = MSG_REPORT_DATA;
				packet_report_data->valid = additional_data->valid;
				packet_report_data->src_addr = packet_std->src;
				packet_report_data->dst_addr = packet_std->dst;
				packet_report_data->sequence_number = packet_data->seq_number;
				packet_report_data->diagnostics = packet_info->diagnostics;
				packet_report_data->length = CIR_LEN;
				packet_report_data->packet_size = packet_info->rx_packet_len;
				memcpy(packet_report_data->CIR, packet_info_with_cir->CIR, CIR_LEN);
				packet_report.length = offsetof(packet_report_data_t, CIR) + packet_report_data->length;

				break;
			case MSG_TWR_FINAL:
				// calculate ToF and report

				// poll_processing_duration; // Db
				// poll_response_duration; // Ra
				// resp_response_duration; // Rb
				// resp_processing_duration; // Da
				// time_of_flight; // (Ra*Rb-Da*Db)/(Ra+Rb+Da+Db)

				packet_ranging_final = (packet_ranging_final_t *)packet_std->payload;
				additional_ranging_final = (additional_ranging_final_t *)packet_info->additional_data;
				packet_report_ranging = (packet_report_ranging_t *)packet_report.data;
				packet_report_ranging->msg_id = MSG_REPORT_TWR;
				packet_report_ranging->local_address = identity_get_address();
				packet_report_ranging->remote_address = packet_std->src;
				packet_report_ranging->ranging_sequence_number = packet_ranging_final->ranging_sequence_number;

				if (packet_ranging_final->twr_resp_rx_ts[identity_get_address()].valid == 1)
				{
					packet_report_ranging->poll_processing_duration = ((((uint64_t)additional_ranging_final->resp_tx_ts) << 8) - additional_ranging_final->poll_rx_ts + UINT40_MAX) % UINT40_MAX;
					packet_report_ranging->poll_response_duration = (packet_ranging_final->twr_resp_rx_ts[identity_get_address()].value - (((uint64_t)packet_ranging_final->twr_poll_tx_ts) << 8) + UINT40_MAX) % UINT40_MAX;
					packet_report_ranging->resp_response_duration = (packet_info->rx_timestamp - (((uint64_t)additional_ranging_final->resp_tx_ts) << 8) + UINT40_MAX) % UINT40_MAX;
					packet_report_ranging->resp_processing_duration = ((((uint64_t)packet_ranging_final->twr_final_tx_ts) << 8) - packet_ranging_final->twr_resp_rx_ts[identity_get_address()].value + UINT40_MAX) % UINT40_MAX;
					packet_report_ranging->time_of_flight = packet_report_ranging->poll_response_duration * packet_report_ranging->resp_response_duration - packet_report_ranging->poll_processing_duration * packet_report_ranging->resp_processing_duration;
					packet_report_ranging->time_of_flight /= (packet_report_ranging->poll_response_duration + packet_report_ranging->resp_response_duration + packet_report_ranging->poll_processing_duration + packet_report_ranging->resp_processing_duration);
					packet_report_ranging->tof_valid = 1;
				}
				else
				{
					packet_report_ranging->poll_processing_duration = 0;
					packet_report_ranging->poll_response_duration = 0;
					packet_report_ranging->resp_response_duration = 0;
					packet_report_ranging->resp_processing_duration = 0;
					packet_report_ranging->time_of_flight = 0;
					packet_report_ranging->tof_valid = 0;
				}

//				packet_report_ranging->poll_tx_timestamp = ((uint64_t)additional_ranging_final->resp_tx_ts) << 8;
				packet_report_ranging->poll_tx_timestamp = ((uint64_t)packet_ranging_final->twr_poll_tx_ts) << 8;
				packet_report_ranging->poll_rx_timestamp = additional_ranging_final->poll_rx_ts;
				packet_report_ranging->final_tx_timestamp = ((uint64_t)packet_ranging_final->twr_final_tx_ts) << 8;
				packet_report_ranging->final_rx_timestamp = packet_info->rx_timestamp;
//					memcpy((uint8_t *)&packet_report_ranging->diagnostics, (uint8_t *)&packet_info->diagnostics, offsetof(packet_diagnostics_t, CIR));
//					memcpy((uint8_t *)&packet_report_ranging->diagnostics.CIR[0], (uint8_t *)&packet_info->diagnostics.CIR[0], CIR_LEN);
				memcpy((uint8_t *)&packet_report_ranging->diagnostics, (uint8_t *)&packet_info->diagnostics, sizeof(packet_diagnostics_t));
//				packet_report_ranging->diagnostics = packet_info->diagnostics;
				packet_report.length = sizeof(packet_report_ranging_t);

				instance_info.diagnostics.twr.report_tx_count++;
				break;
			case MSG_BLINK:
				packet_blink = (packet_blink_t *)packet_std->payload;
				// report blink rx timestamp
				packet_report_blink = (packet_report_blink_t *)packet_report.data;
				packet_report_blink->msg_id = MSG_REPORT_BLINK;
				packet_report_blink->local_address = identity_get_address();
				packet_report_blink->remote_address = packet_std->src;
				packet_report_blink->sequence_number = packet_std->sequence_number;
				packet_report_blink->tx_timestamp = ((uint64_t)packet_blink->tx_timestamp) << 8;
				packet_report_blink->rx_timestamp = packet_info->rx_timestamp;
//				memcpy((uint8_t *)&packet_report_blink->diagnostics, (uint8_t *)&packet_info->diagnostics, offsetof(packet_diagnostics_t, CIR));
//				memcpy((uint8_t *)&packet_report_blink->diagnostics.CIR[0], (uint8_t *)&packet_info->diagnostics.CIR[0], CIR_LEN);
				memcpy((uint8_t *)&packet_report_blink->diagnostics, (uint8_t *)&packet_info_with_cir->diagnostics, sizeof(packet_diagnostics_t));
				memcpy((uint8_t *)&packet_report_blink->CIR[0], (uint8_t *)&packet_info_with_cir->CIR[0], CIR_LEN);
//				packet_report_blink->diagnostics = packet_info->diagnostics;
				packet_report.length = sizeof(packet_report_blink_t);

				instance_info.diagnostics.blink.report_tx_count++;
				break;
			default:
				break;
		}
		r = uwb_rx_swing_buffer.release_read_buffer(buffer);
		if (r == 1)
		{
			instance_info.diagnostics.uwb.rx.swing_buffer_wrong_fingerprint_read_count++;
		}

		// send uart messages
		if (packet_report.length) // if there is anything to send
		{
			r = ring_buffer_write((uint8_t *)&packet_report, packet_report.length + 4);

			if (r == 1)
			{
				instance_info.diagnostics.usart.tx.ring_buffer_full_count++;
			}

			if (ring_buffer_get_length() > instance_info.diagnostics.usart.tx.ring_buffer_max_length)
			{
				instance_info.diagnostics.usart.tx.ring_buffer_max_length = ring_buffer_get_length();
			}

			instance_info.diagnostics.usart.tx.write_count++;
		}
		gpio_low(DEBUG_MAIN);
	}
	else
	{
		// no message is available yet. check back later.
	}
}

static packet_status_t run_state_machine(packet_info_t * packet_info)
{

}

void instance_tx(void)
{

	packet_std_t * packet_std = NULL;
	packet_uuid_t * packet_uuid;
	packet_ranging_poll_t * packet_ranging_poll;
	packet_ranging_response_t * packet_ranging_response;
	packet_ranging_final_t * packet_ranging_final;
	packet_data_t * packet_data = NULL;
	packet_blink_t * packet_blink = NULL;

	uint32_t tx_timestamp = 0;
	uint16_t frame_length = 0;
	uint8_t tx_mode = DWT_START_TX_IMMEDIATE;

	uint8_t * buffer;
	uint8_t r;

	if (state_machine_state == STATE_RX)
	{
		buffer = uwb_tx_swing_buffer.get_read_buffer();
		if (buffer)
		{
			gpio_high(DEBUG_TX);
//			tx_timestamp = dwt_readsystimestamphi32();
//			tx_timestamp += DELAY_TX;
			packet_std = (packet_std_t *)buffer;
			packet_std->packet_id = PACKET_ID;
			packet_std->pan_id = PAN_ID;
			packet_std->sequence_number = instance_info.config.sequence_number++;
			packet_std->src = identity_get_address();
			switch (packet_std->payload[0])
			{
				case MSG_UUID:
					packet_uuid = (packet_uuid_t *)packet_std->payload;
					UNUSED(packet_uuid); // no use case yet
					frame_length = offsetof(packet_std_t, payload) + sizeof(packet_uuid_t);
					tx_mode = DWT_START_TX_IMMEDIATE;
					state_machine_state = STATE_TX;
					break;
				case MSG_DATA:
					packet_data = (packet_data_t *)packet_std->payload;
					packet_data->seq_number = instance_info.config.sequence_number;
					frame_length = HEADER_SIZE(packet_std_t, payload) + HEADER_SIZE(packet_data_t, payload) + packet_data->length;
					tx_mode = DWT_START_TX_IMMEDIATE;
					state_machine_state = STATE_TX;
					break;
				case MSG_TWR_POLL:
					packet_ranging_poll = (packet_ranging_poll_t *)packet_std->payload;
					// keep ranging_sequence_number the same number as the one we send throughout the same ranging session.
					packet_ranging_poll->ranging_sequence_number = ++ranging_sequence_number;
					frame_length = offsetof(packet_std_t, payload) + sizeof(packet_ranging_poll_t);
//					tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;
					tx_mode = DWT_START_TX_DELAYED;

					instance_info.ranging.master.valid = 1;
					instance_info.ranging.master.poll_tx_ts_dw = tx_timestamp;
					instance_info.ranging.master.poll_tx_ts_sys = portGetTickCount();

					instance_info.diagnostics.twr.poll_tx_count++;

					state_machine_state = STATE_TX;
					break;
				case MSG_TWR_RESP:
					packet_ranging_response = (packet_ranging_response_t *)packet_std->payload;
					UNUSED(packet_ranging_response); // no use case yet
					frame_length = offsetof(packet_std_t, payload) + sizeof(packet_ranging_response_t);
//					tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;
					tx_mode = DWT_START_TX_DELAYED;

//					instance_info.ranging.slave.resp_tx_ts_dw.valid = 1;
//					instance_info.ranging.slave.resp_tx_ts_dw.value = tx_timestamp;

					instance_info.diagnostics.twr.resp_tx_count++;

					state_machine_state = STATE_TX;
					break;
				case MSG_TWR_FINAL:
					packet_ranging_final = (packet_ranging_final_t *)packet_std->payload;
					packet_ranging_final->ranging_sequence_number = ranging_sequence_number;
					packet_ranging_final->twr_final_tx_ts = tx_timestamp;
					frame_length = offsetof(packet_std_t, payload) + sizeof(packet_ranging_final_t);
					tx_mode = DWT_START_TX_DELAYED;

					instance_info.diagnostics.twr.final_tx_count++;

					state_machine_state = STATE_TX;
					break;
				case MSG_BLINK:
					packet_blink = (packet_blink_t *)packet_std->payload;
					packet_blink->tx_timestamp = tx_timestamp;
					tx_mode = DWT_START_TX_DELAYED;
					frame_length = offsetof(packet_std_t, payload) + sizeof(packet_blink_t);

					instance_info.diagnostics.blink.tx_count++;

					state_machine_state = STATE_TX;
					break;
				default:
					// unknown packet
					frame_length = 0;
					state_machine_state = STATE_RX;
					break;
			}
		}

		if (state_machine_state == STATE_TX)
		{
			if (frame_length)
			{
				dwt_forcetrxoff();

				dwt_writetxfctrl(frame_length + 2, 0, 0);
//				led_on(LED_RX);
//				dwt_writetxdata(40, (uint8_t *)packet_std, 0);
//				led_off(LED_RX);
				if (tx_mode & DWT_START_TX_DELAYED)
				{
					dwt_setdelayedtrxtime(tx_timestamp);
				}
				led_on(LED_TX);
//				if(identity_get_operations() & IDENTITY_OPERATIONS_ACK_EN)
//					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
//				else
//					dwt_starttx(tx_mode);

				dwt_writetxdata(frame_length + 2, (uint8_t *)(packet_std), 0);
				instance_info.diagnostics.uwb.tx.sent_count++;

				if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
				{
					gpio_low(DEBUG_TX);
					gpio_high(DEBUG_TX);
					gpio_low(DEBUG_TX);
					// start TX was late, TX has been aborted.
					state_machine_state = STATE_RX;
					instance_info.diagnostics.uwb.tx.failed_count++;
				}
//				else
//				{
//

			}
			else
			{
				// don't need to send any packet
			}
		}
		else
		{
			// don't need to send any packet
		}

		if (buffer)
		{
			r = uwb_tx_swing_buffer.release_read_buffer(buffer);
			if (r == 1)
			{
				instance_info.diagnostics.uwb.tx.swing_buffer_wrong_fingerprint_read_count++;
			}
		}
	}
}




/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
int tx_num = 0;
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	led_on(LED_RX);
	uint8_t * buffer = NULL;
	uint8_t r = 0;
	packet_std_t * std_frame = NULL;
	packet_info_t * packet_info = NULL;
	packet_info_with_cir_t *  packet_info_with_cir = NULL;
	packet_diagnostics_t * packet_diagnostics = NULL;
	packet_status_t state_machine_result = PACKET_DROP;
	uint16_t packet_std_header_len = offsetof(packet_std_t, payload) + 1;
	instance_info.diagnostics.uwb.rx.received_count++;
	int32 ci ;
	float clockOffsetHertz;
	float clockOffsetPPM;
	uint16 packet_id = 0;
//	led_off(LED_RX);
    /* A frame has been received, copy it to our local buffer. */
//	if (cb_data->datalength <= FRAME_LEN_MAX + 2)
	if (1)
    {


    	buffer = uwb_rx_swing_buffer.get_write_buffer();
    	r = uwb_rx_swing_buffer.get_free_buffers_length();
    	if (r < instance_info.diagnostics.uwb.rx.swing_buffer_min_free_count)
    	{
    		instance_info.diagnostics.uwb.rx.swing_buffer_min_free_count = r;
    	}
    	r = 0;

//    	led_off(LED_RX);
    	if (1)
    	{
    		packet_info = (packet_info_t *)buffer;
    		uint16 size = 0;
    		packet_info_with_cir = (packet_info_with_cir_t *)buffer;
    		packet_info->rx_packet_len = cb_data->datalength;
    		dwt_readrxdata(packet_info->packet_data, packet_std_header_len, 0);
    		dwt_readrxtimestamp((uint8_t *)&packet_info->rx_timestamp);
    		std_frame = (packet_std_t *)(packet_info->packet_data);
    		packet_id = std_frame->packet_id;
//    		std_frame->packet_size = cb_data->datalength;
    		if (packet_id == PACKET_ID || (packet_id == ACK_ID))
    		{
//    			if (std_frame->dst == identity_get_address() || std_frame->dst == IDENTITY_ADDRESS_BROADCAST || packet_id == ACK_ID)
    			if (1)
    			{
    				state_machine_result = run_state_machine(packet_info);
    				if ((state_machine_result == PACKET_KEEP) | (state_machine_result == PACKET_KEEP_WITH_CIR)){
//    					dwt_readrxdata(packet_info->packet_data + packet_std_header_len, cb_data->datalength - packet_std_header_len, packet_std_header_len);
    					dwt_readrxdata(packet_info->packet_data + packet_std_header_len, 4, packet_std_header_len);
    					// collect RX quality and CIR information
						packet_diagnostics = (packet_diagnostics_t *)&packet_info->diagnostics;
						packet_diagnostics->firstPath = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
						packet_diagnostics->maxNoise = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);
						//read all 8 bytes in one SPI transaction
						dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8*)&packet_diagnostics->stdNoise);
						packet_diagnostics->firstPathAmp1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET);
						packet_diagnostics->rxPreamCount = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT  ;
						packet_diagnostics->rxPreamCountNoSat = dwt_read16bitoffsetreg(DRX_CONF_ID, 0x2C);
						packet_diagnostics->ldeThreshold = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);
						packet_diagnostics->peakPathIndex = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_PPINDX_OFFSET);
						packet_diagnostics->peakPathAmpl = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_PPAMPL_OFFSET);
						packet_diagnostics->ldeCfg1 = dwt_read8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET);
						ci = dwt_readcarrierintegrator();
						if(instance_info.config.radio_config.dataRate == DWT_BR_110K)
							clockOffsetHertz= ci * FREQ_OFFSET_MULTIPLIER_110KB;
						else
							clockOffsetHertz= ci * FREQ_OFFSET_MULTIPLIER;

						switch (instance_info.config.radio_config.chan){
							case 1:
								clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_1;
								break;
							case 2:
							case 4:
								clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_2;
								break;
							case 3:
								clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_3;
								break;
							case 5:
							case 7:
								clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
								break;
						}


						packet_diagnostics->clockOffset = clockOffsetPPM;
//						memset(packet_info_with_cir->packet_data, 100, 10);

//						if (state_machine_result == PACKET_KEEP_WITH_CIR)
						if (1)
						{
#ifdef USE_CIR_FIXED_OFFSET
//							dwt_readaccdata((uint8_t *)&packet_info_with_cir->CIR[0], CIR_LEN, CIR_FIXED_OFFSET);
#else
							dwt_readaccdata((uint8_t *)&packet_info_with_cir->CIR[0], CIR_LEN, (uint16_t)((packet_diagnostics->firstPath >> 6) + (int16_t)CIR_FIRSTPATH_OFFSET)*4);
#endif
							// keep the packet for further processing in main loop.
							r = uwb_rx_swing_buffer.release_write_buffer(buffer, sizeof(packet_info_with_cir_t));

							instance_info.diagnostics.uwb.rx.packets_kept_count++;
						}
						else
						{
							// keep the packet for further processing in main loop.
							led_off(LED_RX);
							size = offsetof(packet_info_t, packet_data);
							r = uwb_rx_swing_buffer.release_write_buffer(buffer, size);

							instance_info.diagnostics.uwb.rx.packets_kept_count++;
						}
    				}
    				else
    				{
    					// packet is marked for deletion, discard it.
    					r = uwb_rx_swing_buffer.release_write_buffer(buffer, 0);
    				}
				}
    			else
    			{
    				// packet is not ours, discard it.
    				r = uwb_rx_swing_buffer.release_write_buffer(buffer, 0);
				}
    		}
    		else
    		{
    			// invalid packet, discard it.
    			r = uwb_rx_swing_buffer.release_write_buffer(buffer, 0);
    		}
    	}
    	else
    	{
    		// swing buffer is full
    		instance_info.diagnostics.uwb.rx.swing_buffer_full_count++;
    	}

    	if (r == 1)
    	{
    		instance_info.diagnostics.uwb.rx.swing_buffer_wrong_fingerprint_write_count++;
    	}
    	else if (r == 2)
		{
			instance_info.diagnostics.uwb.rx.swing_buffer_oversize_write_count++;
		}
    }


	if(identity_get_operations() & IDENTITY_OPERATIONS_ALL_UWB_RX){
		if (identity_get_operations() & IDENTITY_OPERATIONS_ACK_EN){
			packet_data_t * dataPacket = (packet_data_t *) std_frame->payload;
			instance_info.config.sequence_number = dataPacket->seq_number;
			start_data_tx();
			instance_tx();
		}else{
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
	}

//	instance_process_message();
    led_off(LED_RX);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
	instance_info.diagnostics.uwb.rx.to_cb_count++;
	state_machine_state = STATE_RX;
	dwt_forcetrxoff();
	dwt_rxreset();
//	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
	state_machine_state = STATE_RX;
	uint8_t * buffer = NULL;
	uint8_t r = 0;
	packet_info_t * packet_info = NULL;
	packet_info_with_cir_t *  packet_info_with_cir = NULL;
	packet_diagnostics_t * packet_diagnostics = NULL;
	uint16_t packet_std_header_len = offsetof(packet_std_t, payload) + 1;
	packet_std_t * std_frame = NULL;
	int32 ci ;
	float clockOffsetHertz;
	float clockOffsetPPM;
	instance_info.diagnostics.uwb.rx.err_cb_count++;
	buffer = uwb_rx_swing_buffer.get_write_buffer();
	r = uwb_rx_swing_buffer.get_free_buffers_length();
	if (r < instance_info.diagnostics.uwb.rx.swing_buffer_min_free_count)
	{
		instance_info.diagnostics.uwb.rx.swing_buffer_min_free_count = r;
	}
	r = 0;



	packet_info = (packet_info_t *)buffer;
	packet_info_with_cir = (packet_info_with_cir_t *)buffer;
	packet_info->rx_packet_len = cb_data->datalength;
	dwt_readrxdata(packet_info->packet_data, packet_std_header_len + 4, 0);

	dwt_readrxtimestamp((uint8_t *)&packet_info->rx_timestamp);
	std_frame = (packet_std_t *)(packet_info->packet_data);

	packet_diagnostics = (packet_diagnostics_t *)&packet_info->diagnostics;
	packet_diagnostics->firstPath = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
	packet_diagnostics->maxNoise = 0xffff;
	//read all 8 bytes in one SPI transaction
	dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8*)&packet_diagnostics->stdNoise);
	packet_diagnostics->firstPathAmp1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET);
	packet_diagnostics->rxPreamCount = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT  ;
	packet_diagnostics->rxPreamCountNoSat = dwt_read16bitoffsetreg(DRX_CONF_ID, 0x2C);
	packet_diagnostics->ldeThreshold = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);
	packet_diagnostics->peakPathIndex = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_PPINDX_OFFSET);
	packet_diagnostics->peakPathAmpl = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_PPAMPL_OFFSET);
	packet_diagnostics->ldeCfg1 = dwt_read8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET);
	ci = dwt_readcarrierintegrator();
	if(instance_info.config.radio_config.dataRate == DWT_BR_110K)
		clockOffsetHertz= ci * FREQ_OFFSET_MULTIPLIER_110KB;
	else
		clockOffsetHertz= ci * FREQ_OFFSET_MULTIPLIER;

	switch (instance_info.config.radio_config.chan){
		case 1:
			clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_1;
			break;
		case 2:
		case 4:
			clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_2;
			break;
		case 3:
			clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_3;
			break;
		case 5:
		case 7:
			clockOffsetPPM = clockOffsetHertz * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
			break;
	}

	packet_diagnostics->clockOffset = clockOffsetPPM;

#ifdef USE_CIR_FIXED_OFFSET
	dwt_readaccdata((uint8_t *)&packet_info_with_cir->CIR[0], CIR_LEN, CIR_FIXED_OFFSET);
#else
	dwt_readaccdata((uint8_t *)&packet_info_with_cir->CIR[0], CIR_LEN, (uint16_t)((packet_diagnostics->firstPath >> 6) + (int16_t)CIR_FIRSTPATH_OFFSET)*4);
#endif
	// keep the packet for further processing in main loop.
	r = uwb_rx_swing_buffer.release_write_buffer(buffer, sizeof(packet_info_with_cir_t));

	if (identity_get_operations() & IDENTITY_OPERATIONS_ALL_UWB_RX){
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}





/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */

void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
	led_off(LED_TX);
	state_machine_state = STATE_RX;
	instance_info.diagnostics.uwb.tx.conf_cb_count++;
	if(identity_get_operations() & (IDENTITY_OPERATIONS_CONSTANT_TX | IDENTITY_OPERATIONS_PERIODIC_TX)){
		tx_num ++;
		if (tx_num >= instance_info.config.tx_number){
			tx_num = 0;
			packet_report.sync = UART_SYNC;
			packet_report_conf_t *conf_report = (packet_report_conf_t *)packet_report.data;
			conf_report->msg_id = MSG_CONF;
			conf_report->conf_type = MSG_TX_DONE;
			conf_report->length = 4;
			memcpy(conf_report->payload, &instance_info.config.sequence_number, 4);
			instance_info.events.data_tx.enabled = 0;
			packet_report.length = offsetof(packet_report_conf_t, payload) + conf_report->length;
			HAL_UART_Transmit_DMA(&USARTx_Handle, (uint8 *)&packet_report, 12);
//			HAL_UART_Transmit_DMA(&USARTx_Handle, UART_TX_DONE_MSG, sizeof(UART_TX_DONE_MSG));
		}else{
			if(identity_get_operations() & IDENTITY_OPERATIONS_CONSTANT_TX){
				start_data_tx();
				instance_tx();
			}
		}
	}

	if((identity_get_operations() & IDENTITY_OPERATIONS_ACK_EN)){
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}


