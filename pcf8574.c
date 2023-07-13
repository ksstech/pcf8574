/*
 * pcf8574.c Copyright (c) 2023 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_variables.h"

#if (halHAS_PCF8574 > 0)
#include "pcf8574.h"
#include "x_errors_events.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ########################################## Macros ###############################################


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ######################################### Local variables #######################################

u8_t pcf8574Num = 0;
pcf8574_t sPCF8574[halHAS_PCF8574] = { NULL };

// ####################################### Local functions #########################################

/**
 * @brief	Read device, store result in Rbuf
 */
static int pcf8574ReadData(pcf8574_t * psPCF8574) {
	IF_SYSTIMER_START(debugTIMING, stPCF8574);
	int iRV = halI2CM_Queue(psPCF8574->psI2C, i2cR_FB, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574);
	return iRV;
}

/**
 * @brief	Write data from Wbuf to device
 */
static int pcf8574WriteData(pcf8574_t * psPCF8574) {
	IF_SYSTIMER_START(debugTIMING, stPCF8574);
	int iRV = halI2CM_Queue(psPCF8574->psI2C, i2cW, &psPCF8574->Wbuf, sizeof(u8_t), (u8_t *)NULL, 0, (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574);
	return iRV;
}

// ###################################### Global functions #########################################

/**
 * @brief	Set pin direction to be OUTput (0) or INput (1)
 */
void pcf8574DIG_IO_SetDirection(pcf8574_io_t eChan, bool Dir) {
	IF_myASSERT(debugPARAM, eChan < pcf8574NUM_PINS);
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Mask = 1 << (eChan % 8);
	psPCF8574->Mask = Dir ? (psPCF8574->Mask | Mask) : (psPCF8574->Mask & ~Mask);
	pcf8574WriteData(psPCF8574);
}

bool pcf8574DIG_IO_GetState(pcf8574_io_t eChan) {
	IF_myASSERT(debugPARAM, eChan < pcf8574NUM_PINS);
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Pnum = eChan % 8;
	if (psPCF8574->Mask & (1 << Pnum)) {
		pcf8574ReadData(psPCF8574);						// Input pin, read live status
		return (psPCF8574->Rbuf >> Pnum) & 1;
	}
	return (psPCF8574->Wbuf >> Pnum) & 1;
}

void pcf8574DIG_OUT_SetState(pcf8574_io_t eChan, bool NewState, bool Now) {
	IF_myASSERT(debugPARAM, (eChan < pcf8574NUM_PINS))
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Pnum = eChan % 8;
	// Check pin is configured as output
	IF_myASSERT(debugPARAM, ((psPCF8574->Mask >> Pnum) & 1) == 0);
	#if (cmakePLTFRM == HW_KC868A6)
	bool CurState = (psPCF8574->Wbuf >> Pnum) & 1 ? 0 : 1;					// Inverted
	if (NewState != CurState) {
//		P("Chan #%d [%d -> %d]", eChan, CurState, NewState);
		// Active LOW design ie inverted
		psPCF8574->Wbuf = NewState ? psPCF8574->Wbuf & ~(1 << Pnum) : psPCF8574->Wbuf | (1 << Pnum);
		psPCF8574->fDirty = 1;							// bit changed, mark buffer as dirty
		if (Now)
			pcf8574WriteData(psPCF8574);
	}
	#else
	bool CurState = (psPCF8574->Wbuf >> Pnum) & 1 ? 1 : 0;
	if (NewState != CurState) {
		P("Chan #%d [%d -> %d]", eChan, CurState, NewState);
		psPCF8574->Wbuf = NewState ? psPCF8574->Wbuf | (1 << Pnum) : psPCF8574->Wbuf & ~(1 << Pnum);
		psPCF8574->fDirty = 1;							// bit changed, mark buffer as dirty
		if (Now)
			pcf8574WriteData(psPCF8574);
	}
	#endif
}

void pcf8574DIG_OUT_Toggle(pcf8574_io_t eChan, bool Now) {
	IF_myASSERT(debugPARAM, (eChan < pcf8574NUM_PINS))
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Pnum = eChan % 8;
	// Check pin is configured as output
	IF_myASSERT(debugPARAM, ((psPCF8574->Mask >> Pnum) & 1) == 0);
	bool NewState = (psPCF8574->Wbuf >> Pnum) & 1 ? 0 : 1;
	pcf8574DIG_OUT_SetState(eChan, NewState, Now);
}

// ################################## Diagnostics functions ########################################

int	pcf8574Identify(i2c_di_t * psI2C_DI) {
	psI2C_DI->TRXmS	= 10;								// default device timeout
	psI2C_DI->CLKuS = 400;								// Max 13000 (13mS)
	psI2C_DI->Test = 1;									// test mode
	psI2C_DI->Type = i2cDEV_PCF8574;
	psI2C_DI->Speed = i2cSPEED_100;						// does not support 400KHz
	psI2C_DI->DevIdx = pcf8574Num;

	pcf8574_t * psPCF8574 = &sPCF8574[pcf8574Num];
	psPCF8574->psI2C = psI2C_DI;

	// Step 1 - read & write data register
	psPCF8574->Rbuf = 0;
	pcf8574ReadData(psPCF8574);

	// Step 2 - Check initial default values
	if (psPCF8574->Rbuf == 0xFF) {						// passed phase 1, now step 4
		psPCF8574->Wbuf = 0;
		pcf8574WriteData(psPCF8574);
		pcf8574ReadData(psPCF8574);
		if (psPCF8574->Rbuf == psPCF8574->Wbuf) {
			++pcf8574Num;
			psPCF8574->Wbuf = 0xFF;
			pcf8574WriteData(psPCF8574);
			psI2C_DI->Test = 0;
			return erSUCCESS;
		}
	}
	IF_PX(debugTRACK && ioB1GET(ioI2Cinit), "I2C device at 0x%02X not PCF8574", psI2C_DI->Addr);
	psI2C_DI->Test = 0;
	psPCF8574->psI2C = NULL;
	return erFAILURE;
}

int	pcf8574Config(i2c_di_t * psI2C_DI) {
	IF_SYSTIMER_INIT(debugTIMING, stPCF8574, stMICROS, "PCF8574", 200, 3200);
	pcf8574ReConfig(psI2C_DI);
	return erSUCCESS;
}

int	pcf8574Diagnostics(i2c_di_t * psI2C_DI) { return erSUCCESS; }

void pcf8574ReConfig(i2c_di_t * psI2C_DI) { return; }

int pcf8574Report(report_t * psR) {
	int iRV = 0;
	for (u8_t i = 0; i < pcf8574Num; ++i) {
		iRV += halI2C_DeviceReport(psR, (void *) &sPCF8574[i].psI2C);
		iRV += wprintfx(psR, "Rbuf=0x%02hX  Wbuf=0x%02hX  #IRQs=%lu  #Events=%lu\r\n\n",
			sPCF8574[i].Rbuf, sPCF8574[i].Wbuf, xIDI_LostIRQs, xIDI_LostEvents);
	}
	return iRV;
}
#endif
