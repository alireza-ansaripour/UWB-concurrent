/*
 * instance_config.h
 *
 *  Created on: Oct 14, 2019
 *      Author: milad
 */

#ifndef SRC_APPLICATION_INSTANCE_CONFIG_H_
#define SRC_APPLICATION_INSTANCE_CONFIG_H_
#include "instance.h"
#include "deca_device_api.h"
// Anchor 10ms - Tag 10ms Message time
//#define EVENT_UUID_TX_INTERVAL			1000
//#define EVENT_TWR_POLL_TX_INTERVAL		500
//#define EVENT_TWR_RESP_TX_INTERVAL		10
//#define EVENT_TWR_FINAL_TX_INTERVAL		60
//#define EVENT_TWR_FINAL_EXPIRY_INTERVAL	70
//#define EVENT_BLINK_START				80
//#define EVENT_BLINK_TX_INTERVAL			10
//#define EVENT_BLINK_TX_COUNT			5
//#define EVENT_BLINK_TX_AUTO_INCREMENT	180
// Anchor 5ms - Tag 5ms Message time
//#define EVENT_UUID_TX_INTERVAL			1000
//#define EVENT_TWR_POLL_TX_INTERVAL		500
//#define EVENT_TWR_RESP_TX_INTERVAL		5
//#define EVENT_TWR_FINAL_TX_INTERVAL		30
//#define EVENT_TWR_FINAL_EXPIRY_INTERVAL	35
//#define EVENT_BLINK_START				40
//#define EVENT_BLINK_TX_INTERVAL			5
//#define EVENT_BLINK_TX_COUNT			9
//#define EVENT_BLINK_TX_AUTO_INCREMENT	100
// Anchor 10ms - Tag 5ms Message time
#define EVENT_UUID_TX_INTERVAL			1000
#define EVENT_TWR_POLL_TX_INTERVAL		500
#define EVENT_DATA_TX_INTERVAL			100
#define EVENT_TWR_RESP_TX_INTERVAL		10
#define EVENT_TWR_FINAL_TX_INTERVAL		60
#define EVENT_TWR_FINAL_EXPIRY_INTERVAL	70
#define EVENT_BLINK_START				80
#define EVENT_BLINK_TX_INTERVAL			5
#define EVENT_BLINK_TX_COUNT			8
#define EVENT_BLINK_TX_AUTO_INCREMENT	115

#define TAG_BASE_ADDRESS		0x100

//// 100 ms in dw ticks
//#define DELAY_TX 0x17CDC00

//// 5 ms in dw ticks
//#define DELAY_TX 0x130B00

//// 10 ms in dw ticks
//#define DELAY_TX 0x261600

//// 20 ms in dw ticks
//#define DELAY_TX 0x4C2C00

// X ms in dw ticks
#define DELAY_TX_MS(X) (X*0x3CF00)

// X (100 us) in dw ticks
#define DELAY_TX_100US(X) (X*0x6180)

// X (10 us) in dw ticks
#define DELAY_TX_10US(X) (X*0x9C0)

// X (5 us) in dw ticks
#define DELAY_TX_5US(X) (X*0x4E0)

// X us in dw ticks with greatest lower bound estimate (0.9975961538461538 us) -> -2.4038461538461145 ns error
#define DELAY_TX_1US_LOWER(X) (X*0xF9)

// X us in dw ticks with lowest upper bound estimate (1.001602564102564 us) -> 1.602564102564147 ns error
#define DELAY_TX_1US_UPPER(X) (X*0xFA)

// X us in dw ticks with decawave's estimate (1.0256410256410257 us) -> 25.641025641025716 ns error
#define DELAY_TX_1US_DW(X) (X*0x100)

#define DELAY_TX DELAY_TX_100US(25)

void instance_config_identity_init(void);
//void identity_set_role(Role role);

extern dwt_config_t dwt_config;
extern dwt_txconfig_t dwt_txconfig;


#endif /* SRC_APPLICATION_INSTANCE_CONFIG_H_ */
