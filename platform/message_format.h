/*
 * message_format.h
 *
 *  Created on: Jan 18, 2018
 *      Author: milad
 */

#ifndef SRC_PLATFORM_MESSAGE_FORMAT_H_
#define SRC_PLATFORM_MESSAGE_FORMAT_H_

#include <stdint.h>

#define NUM_ANCHORS						6
#define NUM_TAGS						10

#define ADDITIONAL_DATA_LEN_MAX			2

#define FRAME_LEN_MAX					100

// max payload size including CRC		(FRAME_LEN_MAX - ID - SEQ - SRC - DST - FCS - PAN)
#define MAX_PAYLOAD_CRC_SIZE			(FRAME_LEN_MAX -  2 -   1 -   2 -   2 -   2 -   2)
#define MAX_REPORT_DATA_SIZE			5000

// max payload size including CRC		(MAX_PAYLOAD_CRC_SIZE - ID - LEN - SEQ)
#define MAX_DATA_PAYLOAD_SIZE			(MAX_PAYLOAD_CRC_SIZE - 1  -   2 -   4)
// #define MAX_PAYLOAD_READ_SIZE			100

//#define USE_CIR_FIXED_OFFSET
#define CIR_FIXED_OFFSET				0
#define CIR_FIRSTPATH_OFFSET			(-70)
//#define CIR_LEN							(100*4 + 1)
#define CIR_LEN							(0)
#define MAX_CIR_LEN						4065

#define PACKET_ID						0x8861
#define ACK_ID							0x0002
#define PAN_ID							0XDECA

#define MSG_ACK							0xAC
#define MSG_DATA                        0xDA
#define MSG_REPORT_DATA                 0xDB
#define MSG_UUID						0xDD
#define MSG_REPORT_UUID					0xDE
#define MSG_CONF						0xC0
#define MSG_REPORT_TIMESYNC				0x76
#define MSG_TWR_POLL					0x12
#define MSG_TWR_RESP					0x13
#define MSG_TWR_FINAL					0x14
#define MSG_REPORT_TWR					0x15
#define MSG_BLINK						0xB1
#define MSG_REPORT_BLINK				0xB2

#define MSG_TX_DONE						0xD0
#define MSG_CONF_ACK					0xCA

#define UART_SYNC 0x515C

#pragma pack(push, 1)

typedef struct
{
	uint8_t valid;
	uint8_t value;
} uint8v_t;

typedef struct
{
	uint8_t valid;
	uint16_t value;
} uint16v_t;

typedef struct
{
	uint8_t valid;
	uint32_t value;
} uint32v_t;

typedef struct
{
	uint8_t valid;
	uint64_t value;
} uint64v_t;

typedef struct
{
	uint16_t      maxNoise ;          // LDE max value of noise
	uint16_t      firstPathAmp1 ;     // Amplitude at floor(index FP) + 1
	uint16_t      stdNoise ;          // Standard deviation of noise
	uint16_t      firstPathAmp2 ;     // Amplitude at floor(index FP) + 2
	uint16_t      firstPathAmp3 ;     // Amplitude at floor(index FP) + 3
	uint16_t      maxGrowthCIR ;      // Channel Impulse Response max growth CIR
	uint16_t      rxPreamCount ;      // Count of preamble symbols accumulated
	uint16_t      firstPath ;         // First path index (10 bits integer + 6 bits floating point)
	uint16_t      rxPreamCountNoSat;  //RXPACC_NOSAT
	uint16_t      ldeThreshold;
	uint16_t      peakPathIndex;
	uint16_t      peakPathAmpl;
	uint8_t       ldeCfg1;
//	uint8_t       CIR[CIR_LEN];

	float		  clockOffset;
} packet_diagnostics_t;

typedef struct
{
	uint64_t rx_timestamp;
	packet_diagnostics_t diagnostics;
	uint8_t additional_data[ADDITIONAL_DATA_LEN_MAX];
	uint16_t rx_packet_len;
	uint8_t packet_data[FRAME_LEN_MAX];
} packet_info_t;

typedef struct
{
	uint64_t rx_timestamp;
	packet_diagnostics_t diagnostics;
	uint8_t additional_data[ADDITIONAL_DATA_LEN_MAX];
	uint16_t rx_packet_len;
	uint8_t packet_data[FRAME_LEN_MAX];
	uint8_t CIR[CIR_LEN];
} packet_info_with_cir_t;

typedef struct
{
	uint16_t packet_id; // PACKET_ID
	uint8_t  sequence_number;
	uint16_t pan_id; // PAN_ID
	uint16_t dst;
	uint16_t src;
//	uint16_t packet_size;
	uint8_t payload[MAX_PAYLOAD_CRC_SIZE];
} packet_std_t;

typedef struct
{
	uint16_t sync;
	uint16_t length;
	uint8_t data[MAX_REPORT_DATA_SIZE];
} packet_report_t;

typedef struct
{
	uint8_t msg_id; // MSG_ACK
} packet_ack_t;

typedef struct
{
	uint8_t msg_id; // MSG_DATA
	uint32_t seq_number;
	uint16_t length;
	uint8_t payload[MAX_DATA_PAYLOAD_SIZE];
} packet_data_t;

typedef struct
{
	uint8_t valid;
} additional_data_t;

typedef struct
{
	uint8_t msg_id; // MSG_REPORT_DATA
	uint8_t valid;
	uint16_t src_addr;
	uint16_t dst_addr;
	uint32_t sequence_number;
	packet_diagnostics_t diagnostics;
	uint16_t length;
	uint16_t packet_size;
//	uint8_t payload[MAX_DATA_PAYLOAD_SIZE];
	uint8_t CIR[CIR_LEN];
} packet_report_data_t;

typedef struct{
	uint8_t msg_id;
	uint8_t conf_type;
	uint16_t length;
	uint8_t payload[10];
}packet_report_conf_t;

typedef struct
{
	uint8_t msg_id; // MSG_UUID
	uint32_t uuid[3];
} packet_uuid_t;

typedef struct
{
	uint8_t msg_id; // MSG_REPORT_UUID
	uint32_t uuid[3];
} packet_report_uuid_t;

typedef struct
{
	uint8_t msg_id; // MSG_REPORT_TIMESYNC
	uint16_t local_address;
	uint16_t remote_address;
	uint8_t sequence_number;
	uint64_t tx_timestamp;
	uint64_t rx_timestamp;
	packet_diagnostics_t diagnostics;
} packet_report_timesync_t;

typedef struct
{
	uint8_t msg_id; // MSG_TWR_POLL
	uint8_t ranging_sequence_number;
//	uint32_t tx_timestamp;
} packet_ranging_poll_t;

typedef struct
{
	uint8_t msg_id; // MSG_TWR_RESP
	uint8_t ranging_sequence_number;
} packet_ranging_response_t;

typedef struct
{
	uint8_t msg_id; // MSG_TWR_FINAL
	uint8_t ranging_sequence_number;
	uint32_t twr_poll_tx_ts;
	uint64v_t twr_resp_rx_ts[NUM_ANCHORS];
	uint32_t twr_final_tx_ts;
} packet_ranging_final_t;

typedef struct
{
	uint64_t poll_rx_ts;
	uint32_t resp_tx_ts;
} additional_ranging_final_t;

typedef struct
{
	uint8_t msg_id; // MSG_REPORT_TWR
	uint16_t local_address;
	uint16_t remote_address;
	uint8_t ranging_sequence_number;
	uint64_t poll_processing_duration; // Db
	uint64_t poll_response_duration; // Ra
	uint64_t resp_response_duration; // Rb
	uint64_t resp_processing_duration; // Da
	uint64_t time_of_flight; // (Ra*Rb-Da*Db)/(Ra+Rb+Da+Db)
	uint8_t tof_valid;
	uint64_t poll_tx_timestamp;
	uint64_t poll_rx_timestamp;
	uint64_t final_tx_timestamp;
	uint64_t final_rx_timestamp;
	packet_diagnostics_t diagnostics;
} packet_report_ranging_t;

typedef struct
{
	uint8_t msg_id; // MSG_BLINK
	uint32_t tx_timestamp;
} packet_blink_t;

typedef struct
{
	uint8_t msg_id; // MSG_REPORT_BLINK
	uint16_t local_address;
	uint16_t remote_address;
	uint8_t sequence_number;
	uint64_t tx_timestamp;
	uint64_t rx_timestamp;
	packet_diagnostics_t diagnostics;
	uint8_t CIR[CIR_LEN];
} packet_report_blink_t;

#pragma pack(pop)


#endif /* SRC_PLATFORM_MESSAGE_FORMAT_H_ */
