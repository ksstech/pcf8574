// pcf8574.c - Copyright (c) 2023-26 Andre M. Maree / KSS Technologies (Pty) Ltd.

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
#define	debugSTRESS					(debugFLAG & 0x0008)		// compile in the pulse-counter stress self-test

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

#if debugCOUNTERS
	#define incrCOUNTER(c)			if(debugCOUNTERS) ++c
#else
	#define incrCOUNTER(c)
#endif

#if debugSTRESS
// Stress self-test (pcf8574StressTest): drives ALL 6 relays (0x24) simultaneously in one I2C write,
// N cycles, at the fastest rate the relays reliably follow, optionally flooding the bus so each nINT
// read queues behind contention. Maximally stresses the shared-nINT coincident-edge path (ESP32
// edge-loss race #9746 + coalescing). Ground truth: 1 counted pulse per input per cycle. Not wired to
// any key - call pcf8574StressTest() manually (e.g. at startup) when debugSTRESS is enabled.
#define	pcf8574TEST_CYCLES			500				// relay ON/OFF cycles (raise to hunt the rare race; note relay wear)
#define	pcf8574TEST_TON_MS			50				// relay ON  window (>= relay operate time so it reliably closes)
#define	pcf8574TEST_TOFF_MS			50				// relay OFF window (>= relay release time; lower = tougher, verify relays follow)
#define	pcf8574TEST_CONTEND			1				// 1 = flood the I2C bus during each window (max read latency)
#define	pcf8574TEST_CONTEND_N		8				// dummy 0x24 reads per window (<= queue depth headroom)
#endif

// ######################################## Enumerations ###########################################

// ######################################### Structures ############################################

// ######################################### Local constants #######################################

#if (cmakePLTFRM == HW_KC868A6)
gdis_t sPCF8574_Pin = {
	/* nINT is a LEVEL/latched, open-drain signal -> GPIO_INTR_LOW_LEVEL is the correct trigger.
	 * The previous "crash in gpio_isr_loop()" with LOW_LEVEL was the UN-masked level storm; v2
	 * masks the source inside the ISR (halGPIO_IRQdisable) and re-enables it only after the port
	 * read releases nINT, so the storm cannot occur. Edge modes (NEG/POS/ANY) were abandoned: they
	 * lose IRQs via the documented ESP32 edge-race (esp-idf #9746) and the first-IRQ/level wedge. */
	{ 1ULL << GPIO_NUM_13, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_LOW_LEVEL },
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
static volatile u8_t pcf8574IRQactive = 0;				// 1 while a read is in progress
#if debugSTRESS
static volatile u8_t pcf8574TestActive = 0;				// 1 = stress test: count only, suppress event/telemetry notify
static u32_t xIDI_ChanCnt[HAL_IDI] = { 0 };				// per-input leading-edge counter (stress-test observable)
#else
	#define pcf8574TestActive		0					// no stress test -> never suppress the notify
#endif

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
 * @note	i2cW_FB (pri=1, blk=1) to match pca9555Write(): actuator writes go to the FRONT of the queue
 *			and the caller gets the REAL transfer result, not just "queued OK". Blocking is what makes
 *			Flush retry, Verify and Config error handling possible at all; fire-and-forget (i2cW) made
 *			all three untestable. Never called from an ISR (the nINT ISR queues i2cRC directly).
 */
static int pcf8574Write(pcf8574_t * psPCF8574, u8_t U8) {
	return halI2C_Queue(psPCF8574->psI2C, i2cW_FB, &U8, sizeof(u8_t), (u8_t *)NULL, 0, (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
}

// ######################################### IRQ support ###########################################

/**
 *	@brief	Check each input, generate event for every input pulsed
 *	@brief	Called in context of the I2C task (i2cRC completion callback)
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
#if (cmakePLTFRM == HW_KC868A6)
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
#if debugSTRESS
			for (u8_t b = Bit0to1, ch = 0; b; b >>= 1, ++ch) {
				if (b & 1)								// count each input's leading edge at the
					++xIDI_ChanCnt[ch];					// capture layer (pre notify-merge); stress observable
			}
#endif
			if (EventsHandle && !pcf8574TestActive)		// Events ready & not stress-testing? (NULL until
				xTaskNotify(EventsHandle, EventMask, eSetBits);	// vEventsStart@main:182; suppress test flood)
			incrCOUNTER(xIDI_BitsSet);
		} else {
			incrCOUNTER(xIDI_BitsClr);
		}
	} else {
		incrCOUNTER(xIDI_BitsDup);
	}
	if (debugTRACK && (OPT_GET(dbgGDIO) & 2))
		pcf8574ReportStatus(NULL);
	ReadPrv = ReadNow;

	/* nINT is LEVEL-latched and the read above released it. If a further input change is already
	 * pending nINT is still low, so re-arming re-fires the interrupt immediately -> next read. The
	 * level trigger IS the loop: no hand-rolled drain, every pass goes through the queue (fair to the
	 * other devices), and this handler never re-queues (so it cannot recurse via inline-exec). */
#if (cmakePLTFRM == HW_KC868A6)
	pcf8574IRQactive = 0;
	halGPIO_IRQenable(sPCF8574_Pin.pin);				// re-arm the (level) nINT interrupt
#endif
}

/**
 * @brief	Trigger read of the PCF8574, schedule CB to handle the result
 * @param	Arg - index of the PCF8574 that should be read.
 * @note	nINT is LEVEL triggered. Mask the source IMMEDIATELY so the latched low level cannot
 *			re-storm the ISR; it is re-enabled in pcf8574ReadHandler() once the read releases nINT.
 *			The I2C task is guaranteed running (the IRQ is only armed in pcf8574Config(), after the
 *			I2C task is up), so the read can always be queued. Events readiness is handled at notify.
 **/
static void IRAM_ATTR pcf8574IntHandler(void * Arg) {
	halGPIO_IRQdisable(sPCF8574_Pin.pin);				// mask level source (no storm)
	if (pcf8574IRQactive)								// a drain is already scheduled (e.g. shared-ISR
		return;											// re-dispatch while nINT still asserted) -> ignore
	pcf8574IRQactive = 1;
	pcf8574_t * psPCF8574 = &sPCF8574[(int) Arg];
	int iRV = halI2C_Queue(psPCF8574->psI2C, i2cRC, NULL, 0, &psPCF8574->Rbuf, sizeof(u8_t), (i2cq_p1_t)pcf8574ReadHandler, (i2cq_p2_t)Arg);
	if (iRV < erSUCCESS) {								// could not queue (e.g. queue full): recover,
		pcf8574IRQactive = 0;							// don't wedge - re-arm so a later level retries
		halGPIO_IRQenable(sPCF8574_Pin.pin);
		return;
	}
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
		#if (cmakePLTFRM == HW_KC868A6)
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
#if (cmakePLTFRM == HW_KC868A6)						// device specific IN/OUT config
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
#if (cmakePLTFRM == HW_KC868A6)
		if (psI2C->Addr == 0x22) {
			psPCF8574->fInvINP = 1;						// inputs are active-low (opto inverts)
			ESP_ERROR_CHECK(gpio_config(&sPCF8574_Pin.esp32));	// pin=INPUT, intr_type=LOW_LEVEL (not yet armed)
			/* Bootstrap: clear any already-asserted nINT and seed ReadPrv from a REAL reading BEFORE
			 * arming the level IRQ, so the handler has a valid baseline and a pre-asserted latch can
			 * neither be lost nor block pulse #1. (gpio_config sets the type but does not enable
			 * delivery; halGPIO_IRQconfig below enables it via gpio_isr_handler_add.) */
			if (pcf8574ReadData(psPCF8574) == erSUCCESS)
				ReadPrv = (u8_t) (~psPCF8574->Rbuf) & psPCF8574->Mask;
			halGPIO_IRQconfig(sPCF8574_Pin.pin, pcf8574IntHandler, (void *)Idx);	// arm IRQ last
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
		#if (cmakePLTFRM == HW_KC868A6)
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

#if debugSTRESS
// ####################################### Stress self-test ########################################

/**
 * @brief	Pulse-counter stress self-test (KC868-A6 relay->opto loopback rig). Not wired to any key;
 *			call pcf8574StressTest() manually (e.g. at startup) with debugSTRESS enabled.
 *			Drives ALL 6 relays (0x24) simultaneously in ONE I2C write, pcf8574TEST_CYCLES times,
 *			at the fastest rate the relays reliably follow, optionally flooding the bus so each
 *			nINT read is delayed. Ground truth = 1 leading edge per input per cycle. Compares the
 *			per-input capture-layer counts (xIDI_ChanCnt) before/after and reports lost/extra.
 * @note	Undercount => lost pulses (the failure this rework targets). Overcount => relay contact
 *			bounce (expected with mechanical relays; the invariant we assert is "no LOSS"). If ALL
 *			channels undercount uniformly, first suspect relays not following at this speed:
 *			raise pcf8574TEST_TON/TOFF_MS.
 */
int pcf8574StressTest(report_t * psR) {
#if (cmakePLTFRM == HW_KC868A6)
	int outDev = -1;
	for (u8_t i = 0; i < pcf8574Num; ++i)
		if (sPCF8574[i].psI2C->Addr == 0x24) { outDev = i; break; }
	if (outDev < 0)
		return xReport(psR, "StressTest: no 0x24 output device found" strNL);
	pcf8574_t * psOut = &sPCF8574[outDev];

	const u32_t N = pcf8574TEST_CYCLES;
	u32_t base[HAL_IDI];
	for (u8_t ch = 0; ch < HAL_IDI; ++ch) base[ch] = xIDI_ChanCnt[ch];
#if (debugCOUNTERS)
	u32_t bRd = xIDI_IRQread, bSet = xIDI_BitsSet, bDup = xIDI_BitsDup, bYd = xIDI_IRQyield;
#endif
	pcf8574TestActive = 1;								// count at capture layer, suppress event/telemetry notify
	pcf8574DevSetState(outDev, 0x3F, 0);				// ensure all relays OFF to start
	vTaskDelay(pdMS_TO_TICKS(pcf8574TEST_TOFF_MS));

	TickType_t t0 = xTaskGetTickCount();
	for (u32_t c = 0; c < N; ++c) {
		pcf8574DevSetState(outDev, 0x3F, 1);			// ALL 6 relays ON in one write -> coincident edges
#if (pcf8574TEST_CONTEND)
		for (int k = 0; k < pcf8574TEST_CONTEND_N; ++k)	// flood the bus so the nINT read queues behind
			halI2C_Queue(psOut->psI2C, i2cR, NULL, 0, &psOut->Rbuf, sizeof(u8_t), (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
#endif
		vTaskDelay(pdMS_TO_TICKS(pcf8574TEST_TON_MS));
		pcf8574DevSetState(outDev, 0x3F, 0);			// ALL 6 relays OFF
#if (pcf8574TEST_CONTEND)
		for (int k = 0; k < pcf8574TEST_CONTEND_N; ++k)
			halI2C_Queue(psOut->psI2C, i2cR, NULL, 0, &psOut->Rbuf, sizeof(u8_t), (i2cq_p1_t)NULL, (i2cq_p2_t)NULL);
#endif
		vTaskDelay(pdMS_TO_TICKS(pcf8574TEST_TOFF_MS));
	}
	u32_t durMs = (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
	vTaskDelay(pdMS_TO_TICKS(100));						// let the final drain settle (relays now idle)
	pcf8574TestActive = 0;								// resume normal event/telemetry notify

	int iRV = xReport(psR, "PCF8574 stress: cycles=%lu  tON=%d  tOFF=%d  contend=%d(x%d)  dur=%lums" strNL,
			N, pcf8574TEST_TON_MS, pcf8574TEST_TOFF_MS, pcf8574TEST_CONTEND, pcf8574TEST_CONTEND ? pcf8574TEST_CONTEND_N : 0, durMs);
	iRV += xReport(psR, "In   Count  Expect   Error" strNL);
	u32_t totAct = 0; int lost = 0, extra = 0, worstCh = 0; long worstErr = 0;
	for (u8_t ch = 0; ch < HAL_IDI; ++ch) {
		u32_t act = xIDI_ChanCnt[ch] - base[ch];
		long err = (long) act - (long) N;
		iRV += xReport(psR, "%2d  %6lu  %6lu   %+5ld%s" strNL, ch, act, N, err,
				err < 0 ? "  <-- LOST" : (err > 0 ? "  (bounce?)" : "  ok"));
		totAct += act;
		if (err < 0) lost += (int) -err;
		if (err > 0) extra += (int) err;
		if ((err < 0 ? -err : err) > (worstErr < 0 ? -worstErr : worstErr)) { worstErr = err; worstCh = ch; }
	}
	iRV += xReport(psR, "Totals: expect=%lu  actual=%lu  LOST=%d  extra=%d  worst=ch%d(%+ld)" strNL,
			(u32_t)(N * HAL_IDI), totAct, lost, extra, worstCh, worstErr);
#if (debugCOUNTERS)
	iRV += xReport(psR, "Diag d: Read=%lu  Set=%lu  Dup=%lu  Yield=%lu" strNL,
			xIDI_IRQread - bRd, xIDI_BitsSet - bSet, xIDI_BitsDup - bDup, xIDI_IRQyield - bYd);
#endif
	iRV += xReport(psR, "RESULT: %s   (lost=%d, extra=%d)" strNL, lost == 0 ? "PASS (no pulses lost)" : "FAIL (pulses LOST)", lost, extra);
	return iRV;
#else
	return xReport(psR, "StressTest: KC868-A6 only" strNL);
#endif
}
#endif	// debugSTRESS

#endif

/*
 * File history:
 * 2026-07-15  Andre M. Maree
 *   Removed the hand-rolled nINT drain loop (pcf8574IRQ_REARM_MAX / pcf8574IRQrearm and the re-read
 *   chain). With GPIO_INTR_LOW_LEVEL the interrupt mechanism already re-fires while nINT is still
 *   asserted, so re-arming alone suffices: same behaviour, iterative via the queue (fair to the other
 *   I2C devices) instead of a chain that monopolised the I2C task - and the handler no longer calls
 *   halI2C_Queue, so it cannot recurse through the v2 inline-exec path. Level trigger RETAINED: it is
 *   what avoids the ESP32 edge-loss race (esp-idf #9746) and the nINT-stuck-low/no-more-edges wedge.
 *
 * 2026-07-01  Andre M. Maree
 *   PCF8574 nINT (KC868-A6, input @0x22) pulse-capture reliability rework. Scope beyond this file:
 *   halGPIO_IRQenable/disable wrappers (hal_gpio.c/.h); halMemory* + halI2C_GetOp* marked IRAM_ATTR
 *   (hal_memory.c / hal_i2c_common.c); CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y (debug.sdk / release.sdk).
 *   NB: requires the manual U3.pin13(nINT) -> GPIO13 strap.
 *   - IRQ trigger NEGEDGE -> GPIO_INTR_LOW_LEVEL: nINT is level/latched, so level triggering avoids
 *     the documented ESP32 edge-loss race (esp-idf #9746) and the first-IRQ/level wedge.
 *   - pcf8574IntHandler(): mask the level source in-ISR (halGPIO_IRQdisable) to prevent the level
 *     storm; pcf8574IRQactive re-entry guard for shared-ISR re-dispatch; queue-full recovery
 *     re-arms instead of wedging. Boot gate removed (I2C task is always up when the IRQ is armed).
 *   - pcf8574ReadHandler(): notify only when EventsHandle is set (early-boot guard); drain the latch
 *     by re-reading while nINT still low, then re-arm the IRQ. Bounded re-read guard
 *     (pcf8574IRQ_REARM_MAX) protects against a stuck/oscillating line.
 *   - pcf8574Config(): seed ReadPrv + clear nINT via a bootstrap read BEFORE arming the IRQ.
 *   - Added debug counters (xIDI_IRQrearm/Cap) and a debugSTRESS-gated relay->opto stress self-test.
 *   - Cause-fix for an ISR->flash "Cache disabled" panic: the IRAM nINT ISR reaches halI2C_Queue,
 *     whose pure-compute callees are now IRAM (halMemorySRAM/ANY, halI2C_GetOpIdx/Cnt).
 *   TODO (deferred): low-rate safety poll of 0x22 to recover a permanently stuck-low nINT; per-channel
 *     reed debounce only if field over-count appears (the relay stress test showed extra=0).
 */
