// pcf8574.h

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## Enumerations ###########################################

typedef enum {
	#if (HAL_PCF8574 > 0)
	pcf8574IO0,
	pcf8574IO1,
	pcf8574IO2,
	pcf8574IO3,
	pcf8574IO4,
	pcf8574IO5,
	pcf8574IO6,
	pcf8574IO7,
	#endif
	#if (HAL_PCF8574 > 1)
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

typedef enum { cfgDIR, cfgINV, stateGET, stateTGL_LAZY, stateTGL, stateSET_LAZY, stateSET, pcf8574FUNC } pcf8574func_e;

// ######################################### Structures ############################################

struct i2c_di_t;
typedef struct __attribute__((packed)) pcf8574_t {
	struct i2c_di_t * psI2C;
	u8_t Mask;						//	all In=0xFF, all Out=0x00
	u8_t Rbuf;
	u8_t Wbuf;
	u8_t fDirty:1;
	u8_t fInvert:1;
	u8_t spare:6;
} pcf8574_t;
DUMB_STATIC_ASSERT(sizeof(pcf8574_t) == 8);

// ######################################## Global variables #######################################

// ####################################### Global functions ########################################

/**
 * @brief	Check if buffer is dirty, if so write to device.
 * @return	1/true if status written else 0/false
 */
int pcf8574Flush(pcf8574_io_t eChan);

/**
 * @brief
 * @return
 */
int	pcf8574Verify(pcf8574_io_t eChan);

#if (appNEW_CODE == 0)

void pcf8574SetDirection(pcf8574_io_t eCh, bool Dir);

bool pcf8574GetState(pcf8574_io_t eCh);

/**
 * @brief	Update buffered pin status without writing new status to device
 * @return	1/true if buffer is dirty/changed else 0/false
 */
bool pcf8574SetStateLazy(pcf8574_io_t eCh, bool NewState);

/**
 * @brief	Update buffered pin status and [if changed] write status to device
 * @return	1/true if status written else 0/false
 */
bool pcf8574SetState(pcf8574_io_t eCh, bool NewState);

/**
 * @brief	toggle pin status and write status to device
 * @return	pin status after toggled
 */
bool pcf8574Toggle(pcf8574_io_t eCh);

#else

/**
 * @brief	Configure eChan specified as INput
 * @param[in]	eChan on which the operation is to be performed
 * @return	erSUCCESS
 */
//#define pcf8574SetDirection(eChan, Dir)			pcf8574Function(cfgDIR, eChan, Dir)

/**
 * @brief	Configure eChan specified as INVerted INput
 * @param[in]	eChan on which the operation is to be performed
 * @return	erSUCCESS
 */
//#define pcf8574SetInvert(eChan)					pcf8574Function(cfgINV, eChan, 0)

/**
 * @brief	Read the state (0/1) of the eChan specified
 * @param[in]	eChan on which the operation is to be performed
 * @return	boolean state of the eChan
 */
#define pcf8574GetState(eChan)					pcf8574Function(stateGET, eChan, 0)

/**
 * @brief	Toggle the state of a eChan (previously configured as OUTput)
 * @param[in]	eChan on which the operation is to be performed
 * @return	1/true if buffer is dirty/changed else 0/false
 */
//#define pcf8574ToggleLazy(eChan)				pcf8574Function(stateTGL_LAZY, eChan, 0)

/**
 * @brief	Toggle the state of a eChan (previously configured as OUTput)
 * @param[in]	eChan on which the operation is to be performed
 * @return	1/true if status written else 0/false
 */
//#define pcf8574Toggle(eChan)					pcf8574Function(stateTGL, eChan, 0)

/**
 * @brief	Update buffered pin status without writing new status to device
 * @param[in]	eChan on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pcf8574SetStateLazy(eChan, NewState)	pcf8574Function(stateSET_LAZY, eChan, NewState)

/**
 * @brief	Update buffered pin status and [if changed] write status to device
 * @param[in]	eChan on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if status written else 0/false
 */
#define pcf8574SetState(eChan, NewState)		pcf8574Function(stateSET, eChan, NewState)

/**
 * @brief	Perform configuration, read, write and toggle operations on device
 * @param[in]	Func enumerated operation function code
 * @param[in]	eChan virtual channel on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	
 */
int pcf8574Function(pcf8574func_e Func, u8_t eChan, bool NewState);

#endif

//int pcf8574Check(pcf8574_t * psPCF8574);
int	pcf8574Identify(struct i2c_di_t * psI2C);
int	pcf8574Diagnostics(struct i2c_di_t * psI2C);
int	pcf8574Config(struct i2c_di_t * psI2C);
//void pcf8574InitIRQ(void * pvArg);

struct report_t;
int pcf8574Report(struct report_t * psR);

#ifdef __cplusplus
}
#endif
