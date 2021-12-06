/*
 * util.c
 *
 *  Created on: Sep 25, 2020
 *      Author: 18324
 */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"
#include "radio/util.h"
#include "instance.h"

uint16 htons(uint16 s){
	uint16 result = 0;
	result = s % 256;
	result = result << 8;
	result += (s / 256);
	return result;
}

void process_config(uint8 *message, uint16 packet_size){
	dwt_config_t *conf = (dwt_config_t *)(message+1);
	instance_info.config.radio_config = *conf;
	dwt_forcetrxoff();
	dwt_configure(conf);
	if (instance_info.config.enable_64plen_config == 1){
		if(conf->txPreambLength == DWT_PLEN_64){
			led_on(LED_ALL);
			spi_set_rate_low(); //reduce SPI to < 3MHz

			dwt_loadopsettabfromotp(0);

			spi_set_rate_high(); //increase SPI to max
//			dwt_configurefor64plen(conf->prf);
		}else{
			spi_set_rate_low(); //reduce SPI to < 3MHz
			dwt_loadopsettabfromotp(0x2);
			spi_set_rate_high(); //increase SPI to max
		}
	}


	state_machine_state = STATE_RX;
}

void set_role(uint8* message, uint16 packet_len){
	uint8 *data = message + 1;
	uint8 role = (uint8) *data;
	switch(role){
		case ROLE_TX:
			identity_set_role(ROLE_TX);
			break;
		case ROLE_RX:
			identity_set_role(ROLE_RX);
			break;
	}

}

double get_signal_power(){
	uint16 max_cir = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x6);
	uint16 pre_count = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
	double A = 121.74;
	double power = 10 * log10((max_cir * 131072)/pow(pre_count,2)) - A;
	return power;
}

void load_CIR(uint8 *data, uint16 size){
	uint16 first_path = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET) >> 6;
	dwt_readaccdata(data, size, first_path * 4);
}

void change_config(dwt_config_t *conf){
	dwt_forcetrxoff();
	dwt_configure(conf);
}

void start_deleyed_tx(uint32 delay){
	uint32 dlyTxTime = dwt_readsystimestamphi32();
	dlyTxTime += DELAY_TX_100US(400);
	dwt_setdelayedtrxtime(dlyTxTime);
}

static uint32_t tx_power_map[69] =
{
		0xE0E0E0E0, // disable power boost
		0xC0C0C0C0, // 0.0 dB
		0xC1C1C1C1, // 0.5 dB
		0xC2C2C2C2, // 1.0 dB
		0xC3C3C3C3, // 1.5 dB
		0xC4C4C4C4, // 2.0 dB
		0xC5C5C5C5, // 2.5 dB
		0xA0A0A0A0, // 3.0 dB
		0xA1A1A1A1, // 3.5 dB
		0xA2A2A2A2, // 4.0 dB
		0xA3A3A3A3, // 4.5 dB
		0xA4A4A4A4, // 5.0 dB
		0xA5A5A5A5, // 5.5 dB
		0x80808080, // 6.0 dB
		0x81818181, // 6.5 dB
		0x82828282, // 7.0 dB
		0x83838383, // 7.5 dB
		0x84848484, // 8.0 dB
		0xD1D1D1D1, // 8.5 dB
		0x60606060, // 9.0 dB
		0x61616161, // 9.5 dB
		0x62626262, // 10.0 dB
		0x63636363, // 10.5 dB
		0x64646464, // 11.0 dB
		0x65656565, // 11.5 dB
		0x40404040, // 12.0 dB
		0x41414141, // 12.5 dB
		0x42424242, // 13.0 dB
		0x43434343, // 13.5 dB
		0x44444444, // 14.0 dB
		0x45454545, // 14.5 dB
		0x20202020, // 15.0 dB
		0x21212121, // 15.5 dB
		0x22222222, // 16.0 dB
		0x23232323, // 16.5 dB
		0x24242424, // 17.0 dB
		0x25252525, // 17.5 dB
		0x00000000, // 18.0 dB
		0x01010101, // 18.5 dB
		0x02020202, // 19.0 dB
		0x03030303, // 19.5 dB
		0x04040404, // 20.0 dB
		0x05050505, // 20.5 dB
		0x06060606, // 21.0 dB
		0x07070707, // 21.5 dB
		0x08080808, // 22.0 dB
		0x09090909, // 22.5 dB
		0x0A0A0A0A, // 23.0 dB
		0x0B0B0B0B, // 23.5 dB
		0x0C0C0C0C, // 24.0 dB
		0x0D0D0D0D, // 24.5 dB
		0x0E0E0E0E, // 25.0 dB
		0x0F0F0F0F, // 25.5 dB
		0x10101010, // 26.0 dB
		0x11111111, // 26.5 dB
		0x12121212, // 27.0 dB
		0x13131313, // 27.5 dB
		0x14141414, // 28.0 dB
		0x15151515, // 28.5 dB
		0x16161616, // 29.0 dB
		0x17171717, // 29.5 dB
		0x18181818, // 30.0 dB
		0x19191919, // 30.5 dB
		0x1A1A1A1A, // 31.0 dB
		0x1B1B1B1B, // 31.5 dB
		0x1C1C1C1C, // 32.0 dB
		0x1D1D1D1D, // 32.5 dB
		0x1E1E1E1E, // 33.0 dB
		0x1F1F1F1F  // 33.5 dB
};

uint8_t instance_get_tx_power_index(int16_t ten_dB)
{
	if (ten_dB < 0)
	{
		return 0; // disable power boost
	}
	else if ((ten_dB % 5 == 0) && (ten_dB <= 335)) // 0.0 to 33.5 dB
	{
		return ((uint8_t)(ten_dB / 5)) + 1;
	}
	else
	{
		// invalid tx power
		return 0; // disable power boost
	}
}

uint32_t instance_get_tx_power(int16_t ten_dB)
{
	return tx_power_map[instance_get_tx_power_index(ten_dB)];
}

uint32_t instance_get_tx_power_by_index(uint8_t index)
{
	if (index < 69)
	{
		return tx_power_map[index];
	}
	else
	{
		return tx_power_map[0];
	}
}





