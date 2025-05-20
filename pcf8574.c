// pcf8574.c - Copyright (c) 2023-25 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_PCF8574 > 0)
#include "pcf8574.h"
#include "hal_i2c_common.h"
#include "errors_events.h"
#include "report.h"
#include "syslog.h"
#include "systiming.h"
#include "task_events.h"

// ########################################## Macros ###############################################

#define	debugFLAG					0xF004
#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugFLUSH					(debugFLAG & 0x0002)
#define	debugCOUNTERS				(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

#if debugCOUNTERS
	#define incrCOUNTER(c)			if(debugCOUNTERS) ++c
#else
	#define incrCOUNTER(c)
#endif

// ######################################## Enumerations ###########################################

// ######################################### Structures ############################################

// ######################################### Local constants #######################################

#if (appPLTFRM == HW_KC868A6)
gdis_t sPCF8574_Pin = {
//	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_LOW_LEVEL },	// crash in gpio_isr_loop()
//	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_ANYEDGE },	// loses 1st IRQ
//	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_POSEDGE },	// most IRQs lost
//	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_POSEDGE },		// loses 1st IRQ
	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_NEGEDGE },	// loses 1st IRQ
//	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_NEGEDGE },		// loses 1st IRQ, NEW eventually work
	{ GPIO_NUM_13, 0, gdiINT },
};

u8_t pcf8574Cfg[HAL_PCF8574] = {
	0b00111111,						// 0x22 = INputs on 0->5, 6->7 unused 
	0b11000000,						// 0x24 = OUTputs on 0->5, 6->7 unused
};
#else
	#error "Please add required custom support for other platforms where required !!!"
#endif

// ######################################### Local variables #######################################

static u8_t pcf8574Num = 0;

// ######################################## Global variables #######################################

pcf8574_t sPCF8574[HAL_PCF8574] = { NULL };
u8_t ReadPrv, ReadNow, ReadChg, Bit0to1, Bit1to0;
#if (debugCOUNTERS)
	u32_t xIDI_IRQsLost, xIDI_IRQread, xIDI_BitsSet, xIDI_BitsClr, xIDI_BitsDup, xIDI_IRQyield;
#endif

// ##################################### Forward Declarations ######################################

static int pcf8574ReportStatus(report_t * psR);

// ####################################### Basic Read/Write ########################################

/**
 * @brief	Read device, store result in Rbuf
 */
static int pcf8574ReadData(pcf8574_t * psPCF8574) {
	return halI2C_Queue(psPCF8574->psI2C, i2cR_B, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
}

/**
 * @brief	Write data from Wbuf to device
 */
static int pcf8574Write(pcf8574_t * psPCF8574, u8_t U8) {
	return halI2C_Queue(psPCF8574->psI2C, i2cW, &U8, sizeof(u8_t), (u8_t *)NULL, 0, (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
}

// ######################################### IRQ support ###########################################

/**
 *	@brief	Check each input, generate event for every input pulsed
 *	@brief	Called in context of the I2C task
 */
static void pcf8574ReadHandler(void * Arg) {
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
	// Invert, if required, to get real value read
	ReadNow = psPCF8574->fInvINP ? ~psPCF8574->Rbuf : psPCF8574->Rbuf;
	ReadNow &= psPCF8574->Mask;							// Remove OUTput bits
	// If bit0 NOT 1st INput bit, shift RIGHT to remove low order OUTput/0 bits
#if (appPLTFRM == HW_KC868A6)
	// No low order bits used as OUTputs, bits 0->5 consecutive INputs 
#else
	ReadNow >>= __builtin_ctzl((u32_t) psPCF8574->Mask);
#endif
	IF_SYSTIMER_STOP(debugTIMING, stPCF8574);
	if (ReadNow != ReadPrv) {
		ReadChg = ReadPrv ^ ReadNow;					// XOR leave changed (0->1 or 1->0) bits
		Bit0to1 = ReadChg & ReadNow;
		Bit1to0 = ReadChg & ReadPrv;					// required if trailing edge important
		if (Bit0to1) {									// If any leading edge changes
			EventMask = (u32_t) Bit0to1 << evtFIRST_IDI;
			xTaskNotify(EventsHandle, EventMask, eSetBits);
			incrCOUNTER(xIDI_BitsSet);
		} else {
			incrCOUNTER(xIDI_BitsClr);
		}
	} else {
		incrCOUNTER(xIDI_BitsDup);
	}
	if (debugTRACK && (xOptionGet(dbgGDIO) & 2))
		pcf8574ReportStatus(NULL);
	ReadPrv = ReadNow;
}

/**
 * @brief	Trigger read of the PCF8574, schedule CB to handle the result
 * @param	Arg - index of the PCF8574 that should be read.
 **/
static void IRAM_ATTR pcf8574IntHandler(void * Arg) {
	// Since the HW/HAL layer is initialised early during boot process, prior to events 
	// sensors, mqtt or related tasks, IRQs can occur before system is ready to handle them
	#define pcf8574REQ_TASKS	(taskI2C_MASK | taskEVENTS_MASK)
	EventBits_t xEBrun = xEventGroupGetBitsFromISR(TaskRunState);
	if ((xEBrun & pcf8574REQ_TASKS) != pcf8574REQ_TASKS) {
		incrCOUNTER(xIDI_IRQsLost);
		return;
	}
	pcf8574_t * psPCF8574 = &sPCF8574[(int) Arg];
	int iRV = halI2C_Queue(psPCF8574->psI2C, i2cRC, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)pcf8574ReadHandler, (i2cq_p2_t)Arg);
	incrCOUNTER(xIDI_IRQread);
	if (iRV == pdTRUE) {
		incrCOUNTER(xIDI_IRQyield);
		portYIELD_FROM_ISR(); 
	}
	IF_SYSTIMER_START(debugTIMING, stPCF8574);
}

// ###################################### Global functions #########################################

int pcf8574Flush(pcf8574_io_t eChan) {
	int Beg, End, iRV = 0;
	// set range based on parameter
	if (eChan == -1)	{ Beg = 0; End = pcf8574Num-1; }
	else				{ Beg = End = eChan / 8; }
	//loop through range and if dirty, flush
	for (int i = Beg; i <= End; ++i) {
		pcf8574_t * psPCF8574 = &sPCF8574[i];
		if (psPCF8574->fDirty) {
			IF_RPT(debugFLUSH, " [Beg=%d Now=%d End=%d D=x%02X", Beg, i, End, psPCF8574->Wbuf);
			u8_t U8inv = psPCF8574->fInvOUT ? ~psPCF8574->Wbuf : psPCF8574->Wbuf;
			IF_RP(debugFLUSH, "->x%02X", U8inv);
			U8inv &= ~psPCF8574->Mask;
			IF_RP(debugFLUSH, "->x%02X]", U8inv);
			pcf8574Write(psPCF8574, U8inv);
			psPCF8574->fDirty = 0;
			++iRV;
		}
	}
	IF_RP(debugFLUSH && iRV, " iRV=%d" strNL, iRV);
	return iRV;
}

int	pcf8574Verify(pcf8574_io_t eChan) {
#if 0
	int Beg, End, iRV = 0;
	// set range based on parameter
	if (eChan == -1)	{ Beg = 0; End = pcf8574Num; }
	else				{ Beg = End = eChan / 8; }
	//loop through range and if dirty, flush
	for (int i = Beg; i < End; ++i) {
		pcf8574_t * psPCF8574 = &sPCF8574[i];
		// add additional code similar to pca9555Verify()
	}
	return iRV;
#else
	return 0;
#endif
}

int pcf8574BitFunction(pcf8574bit_func_e BitFunc, u8_t eChan, bool NewState) {
	IF_myASSERT(debugPARAM, eChan < pcf8574NUM_PINS && BitFunc < pcf8574BIT_FUNC);
	pcf8574_t * psPCF8574 = &sPCF8574[eChan/8];
	u8_t Mask = 1 << (eChan % 8);
	if (BitFunc >= bitTGL_LAZY) {						// MUST be OUTput type function
		IF_myASSERT(debugTRACK, (psPCF8574->Mask & Mask) == 0);
		bool CurState = psPCF8574->Wbuf & Mask ? 1 : 0;
		if (BitFunc <= bitTGL) {						// bitTGL[_LAZY]
			NewState = !CurState;						// calculate NewState
			BitFunc += (bitSET_LAZY - bitTGL_LAZY);		// bitTGL[_LAZY] -> bitSET[_LAZY]
		}
		if (NewState != CurState) {						// NewState, specified or calculated, different
			if (NewState)	psPCF8574->Wbuf |= Mask;
			else			psPCF8574->Wbuf &= ~Mask;
			psPCF8574->fDirty = 1;						// bit just changed, show as dirty
		}
		return (BitFunc <= bitSET_LAZY) ? psPCF8574->fDirty : pcf8574Flush(eChan);

	} else if (BitFunc == bitGET) {						// Can be INPut or OUTput....
		if (psPCF8574->Mask & Mask) {					// Input pin?
			int iRV = pcf8574ReadData(psPCF8574);		// read live status
			return (iRV == erSUCCESS) ? ((psPCF8574->Rbuf & Mask) ? 1 : 0) : xSyslogError(__FUNCTION__, iRV);
		}
		return (psPCF8574->Wbuf & Mask) ? 1 : 0;		// Initially not correct if last write was mask.

	} else if (BitFunc == bitINV) {						// MUST be INput
		IF_myASSERT(debugTRACK, 0);						// not supported on PCF8574

	} else if (BitFunc == bitDIR) {						// Direction 1=INput vs 0=OUTput
		if (NewState)	psPCF8574->Mask |= Mask;
		else			psPCF8574->Mask &= ~Mask;
		if (NewState)
			pcf8574Write(psPCF8574, psPCF8574->Mask);
	}
	return 0;
}

int pcf8574DevFunction(pcf8574dev_func_e DevFunc, int Dev, u8_t Mask, bool State) {
	IF_myASSERT(debugPARAM, DevFunc < pcf8574DEV_FUNC && Dev < HAL_PCF8574);
	pcf8574_t * psPCF8574 = &sPCF8574[Dev];
	if (DevFunc >= devTGL_LAZY) {						// MUST be OUTput type function
		IF_myASSERT(debugTRACK, (psPCF8574->Mask & Mask) == 0);
		Mask &= ~psPCF8574->Mask;						// restrict mask supplied to only OUTput (0) bits
		if (DevFunc <= devTGL) {						// devTGL[_LAZY]
			psPCF8574->Wbuf ^= Mask;
			DevFunc += (devSET_LAZY - devTGL_LAZY);		// devTGL[_LAZY] -> devSET[_LAZY]
		} else {
			if (State)	psPCF8574->Wbuf |= Mask;
			else		psPCF8574->Wbuf &= ~Mask;
		}
		if (Mask)
			psPCF8574->fDirty = 1;						// some bits changed, show as dirty
		return (DevFunc <= devSET_LAZY) ? psPCF8574->fDirty : pcf8574Flush(Dev*BITS_IN_BYTE);

	} else if (DevFunc == devGET) {						// Can be INPut or OUTput....
		int iRV = pcf8574ReadData(psPCF8574);			// read live status
		return (iRV == erSUCCESS) ? psPCF8574->Rbuf & Mask : xSyslogError(__FUNCTION__, iRV);

	} else if (DevFunc == devINV) {						// MUST be INput
		IF_myASSERT(debugTRACK, 0);						// not supported on PCF8574

	} else if (DevFunc == devDIR) {						// Direction 1=INput vs 0=OUTput
		psPCF8574->Mask = Mask;
		pcf8574Write(psPCF8574, psPCF8574->Mask);
	}
	return 0;
}

// ####################################### Diagnostics #############################################

int	pcf8574Diagnostics(i2c_di_t * psI2C) { myASSERT(0); return erSUCCESS; }

// ############################# Identification and Configuration ##################################

/* KC868A6 specific:
 * PCF8574 at 0x22 is used for input and at 0x24 for outputs.
 * On a device with some/all pins used for inputs, changing input pins at identification stage is likely.
 * These changing inputs will affect the values read hence the test values/mask must be carefully
 * determined. On the KC868A6 only GPIOs 0->5 are used as inputs, so Gpio's 6&7 will be high.
 */
int pcf8574Check(pcf8574_t * psPCF8574) {
	int Count = 0;
	do {
		// Step 1 - write data register
		pcf8574Write(psPCF8574, 0xFF);					// assume inputs to read signature
		// Step 2 - read data register
		psPCF8574->Rbuf = 0;
		int iRV = pcf8574ReadData(psPCF8574);
		if (iRV < erSUCCESS)
			return iRV;
		IF_RP(debugCONFIG, "Addr=x%02X  Read=x%02X" strNL, psPCF8574->psI2C->Addr, psPCF8574->Rbuf);
		// Step 3 - Check device specific pin signature
		#if (appPLTFRM == HW_KC868A6)
			if ((psPCF8574->psI2C->Addr == 0x22) && (psPCF8574->Rbuf & 0xC0) == 0xC0)
				return erSUCCESS;
			if ((psPCF8574->psI2C->Addr == 0x24) && (psPCF8574->Rbuf == 0xFF)) 
				return erSUCCESS;
			SL_ERR("i=%d  A=x%X R=x%02X", iRV, psPCF8574->psI2C->Addr, psPCF8574->Rbuf);
		#endif
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
	int iRV = erINV_STATE;								/* default return error code */
	if (psI2C->IDok == 0)
		goto exit;
	psI2C->CFGok = 0;
	int Idx = psI2C->DevIdx;
	pcf8574_t * psPCF8574 = &sPCF8574[Idx];
#if (appPLTFRM == HW_KC868A6)						// device specific IN/OUT config
	iRV = pcf8574Write(psPCF8574, psPCF8574->Mask = pcf8574Cfg[Idx]);
	if (iRV >= erSUCCESS && psI2C->Addr == 0x24) {
		psPCF8574->Wbuf = 0x00;
		psPCF8574->fDirty = 1;
	}
#endif
	if (iRV < erSUCCESS) {
		halEventUpdateDevice(devMASK_PCF8574, 0);		/* device not functional */
		goto exit;
	}
	// once off init....
	if (psI2C->CFGerr == 0) {
		IF_SYSTIMER_INIT(debugTIMING, stPCF8574, stMICROS, "PCF8574", 1, 100);
#if (appPLTFRM == HW_KC868A6)
		if (psI2C->Addr == 0x22) {
			ESP_ERROR_CHECK(gpio_config(&sPCF8574_Pin.esp32));
			halGPIO_IRQconfig(sPCF8574_Pin.pin, pcf8574IntHandler, (void *)Idx);
			psPCF8574->fInvINP = 1;
		}
		if (psI2C->Addr == 0x24) {
			psPCF8574->fInvOUT = 1;
		}
#endif
	}
	psI2C->CFGok = 1;
	pcf8574Flush(-1);									// ensure any/all dirty buffers flushed
	if (++Idx == pcf8574Num)							// If the last one of these devices
		halEventUpdateDevice(devMASK_PCF8574, 1);		// set the mask to indicate ALL configured
exit:
	return iRV;
}

// ########################################## Reporting ############################################

static int pcf8574ReportStatus(report_t * psR) {
	int iRV = xReport(psR, "\tPrv=x%02X  Now=x%02X  Chg=x%02X  0to1=x%02X  1to0=x%02X", ReadPrv, ReadNow, ReadChg, Bit1to0, Bit0to1);
#if(debugCOUNTERS)
	iRV += xReport(psR, "  Lost=%lu  Read=%lu  Yield=%lu  Set=%lu  Clr=%lu  Dup=%lu",
		xIDI_IRQsLost, xIDI_IRQread, xIDI_IRQyield, xIDI_BitsSet, xIDI_BitsClr, xIDI_BitsDup);
#endif
	return iRV = xReport(psR, fmTST(aNL) ? strNLx2 : strNL);
}

int pcf8574Report(report_t * psR) {
	int iRV = 0;
	for (u8_t i = 0; i < pcf8574Num; ++i) {
		pcf8574_t * psPCF8574 = &sPCF8574[i];
		if (psPCF8574->psI2C->Test)
			pcf8574Check(psPCF8574);
		iRV += halI2C_DeviceReport(psR, (void *) psPCF8574->psI2C);
		iRV += xReport(psR, "\tMask=0x%02hX  Rbuf=0x%02hX  Wbuf=0x%02hX" strNL "\t", psPCF8574->Mask, psPCF8574->Rbuf, psPCF8574->Wbuf);
		#if (appPLTFRM == HW_KC868A6)
		if (i == 0) {
			iRV += halGDI_ReportPin(psR, i, &sPCF8574_Pin, NULL);
			iRV += pcf8574ReportStatus(psR);
		}
		#endif
		if	(psR->sFM.aNL)
			iRV += xReport(psR, strNL);
	}
	return iRV;
}

#endif
