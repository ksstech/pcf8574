/*
 * pcf8574.h - Copyright (c) 2023 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#if (epIDI_IRQ_HANDLER > 0)
	#include "hal_gpio.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## Enumerations ###########################################

typedef enum {
	#if (halHAS_PCF8574 > 0)
	pcf8574IO0,
	pcf8574IO1,
	pcf8574IO2,
	pcf8574IO3,
	pcf8574IO4,
	pcf8574IO5,
	pcf8574IO6,
	pcf8574IO7,
	#endif
	#if (halHAS_PCF8574 > 1)
	pcf8574IO8,
	pcf8574IO9,
	pcf8574IO10,
	pcf8574IO11,
	pcf8574IO12,
	pcf8574IO13,
	pcf8574IO14,
	pcf8574IO15,
	#endif
	pcf8574NUM_PINS
} pcf8574_io_t;

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) pcf8574_t {
	i2c_di_t * psI2C;
	u8_t Mask;						//	all In=0xFF, all Out=0x00
	u8_t Rbuf;
	u8_t Wbuf;
	bool fDirty;
} pcf8574_t;
DUMB_STATIC_ASSERT(sizeof(pcf8574_t) == 8);

// ######################################## Global variables #######################################


// ####################################### Global functions ########################################

void pcf8574ReadTrigger(void * Arg);

void pcf8574DIG_IO_SetDirection(pcf8574_io_t eChan, bool Dir);
bool pcf8574DIG_IO_GetState(pcf8574_io_t eChan);
bool pcf8574DIG_OUT_SetStateLazy(pcf8574_io_t eChan, bool NewState);
bool pcf8574DIG_OUT_SetState(pcf8574_io_t eChan, bool NewState);
void pcf8574DIG_OUT_Toggle(pcf8574_io_t eChan);

int	pcf8574Identify(i2c_di_t * psI2C_DI);
int	pcf8574Diagnostics(i2c_di_t * psI2C_DI);
int	pcf8574Config(i2c_di_t * psI2C_DI);
int pcf8574ReConfig(i2c_di_t * psI2C_DI);
void pcf8574InitIRQ(int PinNum);

int pcf8574Report(report_t * psR);

#ifdef __cplusplus
}
#endif
