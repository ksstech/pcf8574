// pcf8574.c - Copyright (c) 2023-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_PCF8574 > 0)
#include "pcf8574.h"
#include "hal_i2c_common.h"
#include "hal_options.h"
#include "errors_events.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"
#include "task_events.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ########################################## Macros ###############################################

// ######################################## Enumerations ###########################################

// ######################################### Structures ############################################

// ######################################### Local constants #######################################

gdis_t sPCF8574_Pin = {
	{
	#if (buildPLTFRM == HW_KC868A6)
		.pin_bit_mask = (1ULL << pcf8574DEV_0_IRQ),
	#else
		.pin_bit_mask = 0,
		#warning "Please add default values for new platform"
	#endif
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE, 
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE,
	},
	{
	#if (buildPLTFRM == HW_KC868A6)
		.pin = pcf8574DEV_0_IRQ, .inv = 0, .type = gdiINT,
	#else
		.pin = GPIO_NUM_NC, .inv = 0, .type = gdiPIN,
	#endif
	},
};

// ######################################### Local variables #######################################

static u8_t pcf8574Num = 0;
u8_t pcf8574Cfg[HAL_PCF8574] = {
#if (buildPLTFRM == HW_KC868A6)
	0b11111111,						// INputs on 0->7 although only 0->5 used
	0b11000000,						// OUTputs on 0->5, Unused (INputs) on 6->7
#else
	#error "Please add default values for new platform"
#endif
};

// ######################################## Global variables #######################################

u8_t ReadPrv, ReadNow, ReadChg, Bit0to1, Bit1to0;
u32_t xIDI_IRQsLost, xIDI_IRQread, xIDI_BitsSet, xIDI_BitsClr, xIDI_BitsDup, xIDI_IRQyield;
pcf8574_t sPCF8574[HAL_PCF8574] = { NULL };

// ####################################### Local functions #########################################

int pcf8574ReportStatus(report_t * psR) { 
	return wprintfx(psR, "L=%lu R=%lu Y=%lu S=%lu C=%lu D=%lu - P=x%02X  N=x%02X  C=x%02X  0=x%02X  1=x%02X" strNL, xIDI_IRQsLost,
		xIDI_IRQread, xIDI_IRQyield, xIDI_BitsSet, xIDI_BitsClr, xIDI_BitsDup, ReadPrv, ReadNow, ReadChg, Bit1to0, Bit0to1);
}

/**
 * @brief	Read device, store result in Rbuf
 */
static int pcf8574ReadData(pcf8574_t * psPCF8574) {
	int iRV = halI2C_Queue(psPCF8574->psI2C, i2cR_B, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
	return iRV;
}

/**
 * @brief	Write data from Wbuf to device
 */
static int pcf8574WriteData(pcf8574_t * psPCF8574) {
	return halI2C_Queue(psPCF8574->psI2C, i2cW, &psPCF8574->Wbuf, sizeof(u8_t), (u8_t *)NULL, 0, (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
}

static int pcf8574WriteMask(pcf8574_t * psPCF8574) {
	psPCF8574->Wbuf = psPCF8574->Mask;
	return pcf8574WriteData(psPCF8574);
}

/**
 *	@brief	Check each input, generate event for every input pulsed
 *	@brief	Called in context of the I2C task
 */
void pcf8574ReadHandler(void * Arg) {
	/* ReadPrv	--------00010000	Previous reading after all corrections
	 *
	 * Read		--------11110111	Bit2 & 3 input pulled low, ie switch closed ie pulse
	 * Invert	--------00001000	1 = closed
	 * Masked	--------00001000	with 00111111 to remove unused/OUTput bits
	 * Shift R	--------00001000	not required, bottom 6 bits INputs
	 * ReadNow	--------00001000	
	 * Compare						If same as previous reading ignore, nothing changed
	 * Changed	--------00011000	XOR with ReadPrv leaves changed bits
	 * 
	 * Bit1to0	--------00010000	AND with ReadPrv leaves 1->0 changes (Not used now)
	 * Bit0to1	--------00001000	AND with ReadNow leaves 0->1 changes
	 * EventMask-------000011000	shift left with evtFIRST_IDI = 1
	 */
	u8_t eDev = (int) Arg;
	IF_myASSERT(debugTRACK, eDev < HAL_PCF8574);
	pcf8574_t * psPCF8574 = &sPCF8574[eDev];
	u32_t EventMask = 0;
	ReadNow = ~psPCF8574->Rbuf;							// Invert to get real value read
	ReadNow &= psPCF8574->Mask;							// Remove OUTput bits
	// If bit0 NOT 1st INput bit value should be shifted RIGHT to remove low order OUTput bits
	ReadNow >>= __builtin_ctzl((u32_t) psPCF8574->Mask);	// Remove low order output (0) bits (if any)
	if (ReadNow != ReadPrv) {
		ReadChg = ReadPrv ^ ReadNow;					// XOR leave changed (0->1 or 1->0) bits
		Bit0to1 = ReadChg & ReadNow;
		Bit1to0 = ReadChg & ReadPrv;					// not used, only if trailing edge important
		if (Bit0to1) {									// If any leading edge changes
			EventMask = (u32_t) Bit0to1 << evtFIRST_IDI;
			xTaskNotify(EventsHandle, EventMask, eSetBits);
			++xIDI_BitsSet;
		} else {
			++xIDI_BitsClr;
		}
	} else {
		++xIDI_BitsDup;
	}
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574B);
	if (debugTRACK && (ioB2GET(dbgGDIO) & 2)) pcf8574ReportStatus(NULL);
	ReadPrv = ReadNow;
}

/**
 * @brief	Trigger read of the PCF8574, schedule CB to handle the result
 * @param	Arg - index of the PCF8574 that should be read.
 **/
void IRAM_ATTR pcf8574IntHandler(void * Arg) {
	#define pcf8574REQ_TASKS	(taskI2C_MASK | taskEVENTS_MASK)
	EventBits_t xEBrun = xEventGroupGetBitsFromISR(TaskRunState);
	// Since the HW/HAL layer is initialised early during boot process, prior to events 
	// sensors, mqtt or related tasks, IRQs can occur before system is ready to handle them
	if ((xEBrun & pcf8574REQ_TASKS) != pcf8574REQ_TASKS) {
		++xIDI_IRQsLost;
		return;
	}
	u8_t eDev = (int) Arg;
	IF_myASSERT(debugTRACK && buildPLTFRM == HW_KC868A6, eDev == 0);
	pcf8574_t * psPCF8574 = &sPCF8574[eDev];
	int iRV = halI2C_Queue(psPCF8574->psI2C, i2cRC, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)pcf8574ReadHandler, (i2cq_p2_t)Arg);
	++xIDI_IRQread;
	if (iRV == pdTRUE) {
		++xIDI_IRQyield;
		portYIELD_FROM_ISR(); 
	}
}

// ################################## Diagnostics functions ########################################

/* KC868A6 specific:
 * PCF8574 at 0x22 is used for input and at 0x24 for outputs.
 * On a device with some/all pins used for inputs,changing input pins at identification stage is likely.
 * These changing inputs will affect the values read hence the test values/mask must be carefully
 * determined. ON the KC868A6 only GPIO's 0 to 5 are used as inputs, so Gpio's 6&7 will be high.
 */
int pcf8574Check(pcf8574_t * psPCF8574) {
	int Count = 0;
	do {
		// Step 1 - read data register
		psPCF8574->Rbuf = 0;
	#if (buildPLTFRM == HW_KC868A6)
		if (psPCF8574->psI2C->Addr == 0x24) {
			psPCF8574->Wbuf = pcf8574Cfg[psPCF8574->psI2C->DevIdx];
			pcf8574WriteData(psPCF8574);
		}
	#endif
		int iRV = pcf8574ReadData(psPCF8574);
		if (iRV < erSUCCESS)
			return iRV;
		// Step 2 - Check initial default values, should be all 1's after PowerOnReset
	#if (buildPLTFRM == HW_KC868A6)
		if ((psPCF8574->psI2C->Addr == 0x22) && (psPCF8574->Rbuf & 0xC0) == 0xC0)
			return erSUCCESS;
		if ((psPCF8574->psI2C->Addr == 0x24) && (psPCF8574->Rbuf == 0xFF)) 
			return erSUCCESS;
		SL_ERR("i=%d  A=x%X R=x%02X", iRV, psPCF8574->psI2C->Addr, psPCF8574->Rbuf);
	#else
		#error "Custom test code required for this platform"
	#endif
		psPCF8574->Wbuf = 0xFF;							// set all 1's = Inputs
		pcf8574WriteData(psPCF8574);
	} while(++Count < 5);
	return erINV_WHOAMI;
}

int	pcf8574Identify(i2c_di_t * psI2C) {
	pcf8574_t * psPCF8574 = &sPCF8574[pcf8574Num];
	psPCF8574->psI2C = psI2C;
	psI2C->Type = i2cDEV_PCF8574;
	psI2C->Speed = i2cSPEED_100;						// does not support 400KHz
	psI2C->TObus = 5;									// was 100
	psI2C->Test = 1;
	int iRV = pcf8574Check(psPCF8574);
	if (iRV < erSUCCESS)
		goto exit;
	psI2C->DevIdx = pcf8574Num++;						// mark as identified
	psI2C->IDok = 1;
	psI2C->Test = 0;
exit:
	return iRV;
}

int	pcf8574Config(i2c_di_t * psI2C) {
	if (!psI2C->IDok)
		return erINV_STATE;
	psI2C->CFGok = 0;
	pcf8574_t * psPCF8574 = &sPCF8574[psI2C->DevIdx];
	psPCF8574->Mask = pcf8574Cfg[psI2C->DevIdx];
	int iRV = pcf8574WriteMask(psPCF8574);
	if (iRV < erSUCCESS)
		goto exit;
	psI2C->CFGok = 1;
	// once off init....
	if (psI2C->CFGerr == 0) {
		IF_SYSTIMER_INIT(debugTIMING, stPCF8574A, stMICROS, "PCF8574A", 1, 100);
		IF_SYSTIMER_INIT(debugTIMING, stPCF8574B, stMICROS, "PCF8574B", 500, 10000);
	#if (buildPLTFRM == HW_KC868A6)
		if (psI2C->Addr == 0x22) {
			ESP_ERROR_CHECK(gpio_config(&sPCF8574_Pin.esp32));
			halGPIO_IRQconfig(sPCF8574_Pin.pin, pcf8574IntHandler, (void *)(int)psI2C->DevIdx);
		}
	#else
		#warning " Add IRQ support if required."
	#endif
	}
exit:
	return iRV;
}

// ################################## Diagnostics functions ########################################

int	pcf8574Diagnostics(i2c_di_t * psI2C) { myASSERT(0); return erSUCCESS; }

// ###################################### Global functions #########################################

/**
 * @brief	Set pin direction to be OUTput (0) or INput (1)
 */
void pcf8574DIG_IO_SetDirection(pcf8574_io_t eChan, bool Dir) {
	IF_myASSERT(debugPARAM, eChan < pcf8574NUM_PINS);
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Mask = 1 << (eChan % 8);
	// Set correct bit in mask to indicate direction
	psPCF8574->Mask = Dir ? (psPCF8574->Mask | Mask) : (psPCF8574->Mask & ~Mask);
	if (Dir)
		pcf8574WriteMask(psPCF8574);					// If set as INput, write mask to device
}

bool pcf8574DIG_IO_GetState(pcf8574_io_t eChan) {
	IF_myASSERT(debugPARAM, eChan < pcf8574NUM_PINS);
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Pnum = eChan % 8;
	if (psPCF8574->Mask & (1 << Pnum)) {
		pcf8574ReadData(psPCF8574);						// Input pin, read live status
		return (psPCF8574->Rbuf >> Pnum) & 1;
	}
	return (psPCF8574->Wbuf >> Pnum) & 1;				// Initially not correct if last write was mask.
}

bool pcf8574DIG_OUT_SetStateLazy(pcf8574_io_t eChan, bool NewState) {
	IF_myASSERT(debugPARAM, (eChan < pcf8574NUM_PINS))
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Pnum = eChan % 8;
	// Check pin is configured as output
	IF_myASSERT(debugPARAM, ((psPCF8574->Mask >> Pnum) & 1) == 0);
	bool CurState = (psPCF8574->Wbuf >> Pnum) & 1 ? 0 : 1;	// Active LOW ie inverted
	bool bRV = NewState != CurState ? 1 : 0;
	if (bRV) {
		psPCF8574->Wbuf = NewState ? psPCF8574->Wbuf & ~(1 << Pnum) : psPCF8574->Wbuf | (1 << Pnum);
		psPCF8574->fDirty = 1;							// bit changed, mark buffer as dirty
	}
	return bRV;
}

bool pcf8574DIG_OUT_SetState(pcf8574_io_t eChan, bool NewState) {
	int bRV = pcf8574DIG_OUT_SetStateLazy(eChan, NewState);
	if (bRV) {
		pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
		pcf8574WriteData(psPCF8574);
	}
	return bRV;
}

void pcf8574DIG_OUT_Toggle(pcf8574_io_t eChan) {
	IF_myASSERT(debugPARAM, (eChan < pcf8574NUM_PINS))
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Pnum = eChan % 8;
	// Check pin is configured as output
	IF_myASSERT(debugPARAM, ((psPCF8574->Mask >> Pnum) & 1) == 0);
	bool NewState = (psPCF8574->Wbuf >> Pnum) & 1 ? 0 : 1;
	pcf8574DIG_OUT_SetState(eChan, NewState);
}

int pcf8574Report(report_t * psR) {
	int iRV = 0;
	for (u8_t i = 0; i < pcf8574Num; ++i) {
		pcf8574_t * psPCF8574 = &sPCF8574[i];
		if (psPCF8574->psI2C->Test)					pcf8574Check(psPCF8574);
		iRV += halI2C_DeviceReport(psR, (void *) psPCF8574->psI2C);
		iRV += wprintfx(psR, "Mask=0x%02hX  Rbuf=0x%02hX  Wbuf=0x%02hX  ", psPCF8574->Mask, psPCF8574->Rbuf, psPCF8574->Wbuf);
		#if (buildPLTFRM == HW_KC868A6)
		if (i == 0) {
			bool fSave = psR->sFM.aNL;
			psR->sFM.aNL = 0;
			iRV += halGDI_ReportPin(psR, i, &sPCF8574_Pin, NULL);
			psR->sFM.aNL = fSave;
			iRV += pcf8574ReportStatus(psR);
		}
		#endif
		if	(psR->sFM.aNL) iRV += wprintfx(psR, strNL);
	}
	return iRV;
}
#endif
