/*
 * pcf8574.h - Copyright (c) 2023 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

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

struct i2c_di_t;
typedef struct __attribute__((packed)) pcf8574_t {
	struct i2c_di_t * psI2C;
	u8_t Mask;						//	all In=0xFF, all Out=0x00
	u8_t Rbuf;
	u8_t Wbuf;
	u8_t IRQpin:6;
	u8_t fDirty:1;
} pcf8574_t;
DUMB_STATIC_ASSERT(sizeof(pcf8574_t) == 8);

// ######################################## Global variables #######################################


// ####################################### Global functions ########################################

void pcf8574ReadTrigger(void * Arg);

void pcf8574DIG_IO_SetDirection(pcf8574_io_t eCh, bool Dir);
bool pcf8574DIG_IO_GetState(pcf8574_io_t eCh);
bool pcf8574DIG_OUT_SetStateLazy(pcf8574_io_t eCh, bool NewState);
bool pcf8574DIG_OUT_SetState(pcf8574_io_t eCh, bool NewState);
void pcf8574DIG_OUT_Toggle(pcf8574_io_t eCh);

int pcf8574Check(pcf8574_t * psPCF8574);
int	pcf8574Identify(struct i2c_di_t * psI2C);
int	pcf8574Diagnostics(struct i2c_di_t * psI2C);
int	pcf8574Config(struct i2c_di_t * psI2C);
void pcf8574InitIRQ(int PinNum);
struct report_t;
int pcf8574Report(struct report_t * psR);

#ifdef __cplusplus
}
#endif
