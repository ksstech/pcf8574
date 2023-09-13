/*
 * pcf8574.c Copyright (c) 2023 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_variables.h"

#if (halHAS_PCF8574 > 0)
#include "hal_i2c_common.h"
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

static u8_t pcf8574Num = 0;
u8_t pcf8574Cfg[halHAS_PCF8574] = {
	#if (cmakePLTFRM == HW_KC868A6)
	0b11111111,						// INputs on 0->7 although only 0->5 used
	0b11000000,						// OUTputs on 0->6, Unused (INputs) on 6->7
	#else
	#error "Please add default values for new platform"
	#endif
};

// ######################################## Global variables #######################################

u32_t xIDI_IRQsOK, xIDI_IRQsLost, xIDI_IRQsHdld, xIDI_IRQsIgnr;
pcf8574_t sPCF8574[halHAS_PCF8574] = { NULL };

// ####################################### Local functions #########################################

/**
 * @brief	Read device, store result in Rbuf
 */
static int pcf8574ReadData(pcf8574_t * psPCF8574) {
	IF_SYSTIMER_START(debugTIMING, stPCF8574);
	int iRV = halI2C_Queue(psPCF8574->psI2C, i2cR_B, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574);
	return iRV;
}

/**
 * @brief	Write data from Wbuf to device
 */
static int pcf8574WriteData(pcf8574_t * psPCF8574) {
	IF_SYSTIMER_START(debugTIMING, stPCF8574);
	int iRV = halI2C_Queue(psPCF8574->psI2C, i2cW, &psPCF8574->Wbuf, sizeof(u8_t), (u8_t *)NULL, 0, (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574);
	return iRV;
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
	/* Read		--------00110000
	 * Invert	--------11001111
	 * Mask		--------11001100	with 11111100
	 * Shift R	--------00110011	lowest 2 bits are outputs
	 * Shift L	-------001100110	evtFIRST_IDI = 1
	 *
	 * If bit0 NOT 1st INput bit value should be shifted RIGHT to remove low order OUTput bits
	 */
	u8_t eDev = (int) Arg;
	IF_myASSERT(debugTRACK, eDev < halHAS_PCF8574);
	pcf8574_t * psPCF8574 = &sPCF8574[eDev];
	u8_t Mask = ~psPCF8574->Rbuf;						// Convert active LOW to HIGH
	Mask &= psPCF8574->Mask;							// Remove OUTput bits
	Mask >>= __builtin_ctzl((u32_t) psPCF8574->Mask);
	u32_t EventMask;
	if (Mask) {
		EventMask = (u32_t) Mask << evtFIRST_IDI;
		xTaskNotifyFromISR(EventsHandle, EventMask, eSetBits, NULL);
		++xIDI_IRQsHdld;
	} else {
		EventMask = 0;
		++xIDI_IRQsIgnr;
	}
	IF_PT(debugTRACK && (ioB2GET(dbgGPI) & 2), "0x%02X->0x%08X (L=%d H=%d I=%d OK=%d)\r\n",
		psPCF8574->Rbuf, EventMask, xIDI_IRQsLost, xIDI_IRQsHdld, xIDI_IRQsIgnr, xIDI_IRQsOK);
}

void pcf8574ReadTrigger(void * Arg) {
	u8_t eDev = (int) Arg;
	pcf8574_t * psPCF8574 = &sPCF8574[eDev];
	IF_SYSTIMER_START(debugTIMING, stPCF8574);
	halI2C_Queue(psPCF8574->psI2C, i2cRC_F, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)pcf8574ReadHandler, (i2cq_p2_t)Arg);
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574);
}

/**
 * @brief	Trigger read of the PCF8574, schedule CB to handle the result
 * @param	Index of the PCF8574 that should be read.
 **/
void IRAM_ATTR pcf8574IntHandler(void * Arg) {
	/* Since the HW/HAL layer is initialised early during boot process, prior
	 * to events/sensors/mqtt/related tasks, IRQs can occur before the system
	 * is ready to handle them.
	 */
	#define pcf8574REQ_TASKS	(taskI2C_MASK | taskEVENTS_MASK)
	EventBits_t xEBrun = xEventGroupGetBitsFromISR(TaskRunState);
	if ((xEBrun & pcf8574REQ_TASKS) == pcf8574REQ_TASKS) {
		u8_t eDev = (int) Arg;
		xTaskNotifyFromISR(EventsHandle, 1UL << eDev, eSetBits, NULL);
		++xIDI_IRQsOK;
	} else {
		++xIDI_IRQsLost;
	}
}

void pcf8574InitIRQ(int PinNum) {
	const gpio_config_t int_pin_cfg = {
		.pin_bit_mask = 1ULL << PinNum, .mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&int_pin_cfg));
	halGPIO_IRQconfig(PinNum, pcf8574IntHandler, (void *) epIDI_DEV_INDEX);
}

// ################################## Diagnostics functions ########################################

/* KC868A6 specific:
 * PCF8574 at 0x22 is used for input and at 0x24 for outputs.
 * On a device used for inputs, all or some pins, changing inputs at identification stage is likely.
 * These changing inputs will affect the values read hence the test values/mask must be carefully
 * determined. ON the KC868A6 only GPIO's 0 to 5 are used as inputs, so Gpio's 6&7 will be high.
 */
int pcf8574Check(pcf8574_t * psPCF8574) {
	int iRV = 0;
	do {
		// Step 1 - read data register
		psPCF8574->Rbuf = 0;
		pcf8574ReadData(psPCF8574);
		// Step 2 - Check initial default values, should be all 1's after PowerOnReset
		#if (cmakePLTFRM == HW_KC868A6)
		if ((psPCF8574->psI2C->Addr == 0x22) && (psPCF8574->Rbuf & 0xC0) == 0xC0) return erSUCCESS;
		if ((psPCF8574->psI2C->Addr == 0x24) && (psPCF8574->Rbuf == 0xFF)) return erSUCCESS;
		RP("i=%d  A=x%X R=x%02X", iRV, psPCF8574->psI2C->Addr, psPCF8574->Rbuf);
		#else
			#error "Custom test code required for this platform"
		#endif
		psPCF8574->Wbuf = 0xFF;						// set all 1's = Inputs
		pcf8574WriteData(psPCF8574);
		++iRV;
	} while(iRV < 5);
	return erFAILURE;
}

int	pcf8574Identify(i2c_di_t * psI2C) {
	psI2C->TRXmS = 10;									// default device timeout
	psI2C->CLKuS = 400;									// Max 13000 (13mS)
	psI2C->Test = 1;									// test mode
	psI2C->Type = i2cDEV_PCF8574;
	psI2C->Speed = i2cSPEED_100;						// does not support 400KHz
	psI2C->DevIdx = pcf8574Num;

	pcf8574_t * psPCF8574 = &sPCF8574[pcf8574Num];
	psPCF8574->psI2C = psI2C;
	int iRV = pcf8574Check(psPCF8574);
	if (iRV == erSUCCESS) {
		++pcf8574Num;									// mark as identified
		psI2C->Test = 0;
		return erSUCCESS;
	}
	SL_ERR(" Failed after %d retries", iRV);
	IF_PX(debugTRACK && ioB1GET(ioI2Cinit), "I2C device at 0x%02X not PCF8574", psI2C->Addr);
//	psI2C->Test = 0;
//	psPCF8574->psI2C = NULL;
	return erFAILURE;
}

int	pcf8574Config(i2c_di_t * psI2C) {
	IF_SYSTIMER_INIT(debugTIMING, stPCF8574, stMICROS, "PCF8574", 200, 3200);
	int iRV = pcf8574ReConfig(psI2C);
	#if (cmakePLTFRM == HW_KC868A6)
	if (psI2C->Addr == 0x22) pcf8574InitIRQ(sPCF8574[psI2C->DevIdx].IRQpin = pcf8574DEV_0_IRQ);
	else sPCF8574[psI2C->DevIdx].IRQpin = -1;
	#else
	#warning " Add IRQ support if required."
	#endif
	return iRV;
}

int pcf8574ReConfig(i2c_di_t * psI2C) {
	pcf8574_t * psPCF8574 = &sPCF8574[psI2C->DevIdx];
	psPCF8574->Mask = pcf8574Cfg[psI2C->DevIdx];
	int iRV = pcf8574WriteMask(psPCF8574);
	if (iRV > erFAILURE) xEventGroupSetBits(EventDevices, devMASK_PCF8574);
	return iRV;
}

// ################################## Diagnostics functions ########################################

int	pcf8574Diagnostics(i2c_di_t * psI2C) { return erSUCCESS; }

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
	if (Dir) pcf8574WriteMask(psPCF8574);	// If set as INput, write mask to device
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
		if (psPCF8574->psI2C->Test) pcf8574Check(psPCF8574);
		iRV += halI2C_DeviceReport(psR, (void *) psPCF8574->psI2C);
		iRV += wprintfx(psR, "Mask=0x%02hX  Rbuf=0x%02hX  Wbuf=0x%02hX", psPCF8574->Mask, psPCF8574->Rbuf, psPCF8574->Wbuf);
		if (psPCF8574->IRQpin == pcf8574DEV_0_IRQ)
			iRV += wprintfx(psR, "  IRQs  L=%lu  H=%lu  I=%lu  OK=%lu", xIDI_IRQsLost, xIDI_IRQsHdld, xIDI_IRQsIgnr, xIDI_IRQsOK);
		iRV += wprintfx(psR, strCRLF);
	}
	return iRV;
}
#endif
