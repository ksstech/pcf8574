// pcf8574.h

#pragma once

#include "definitions.h"

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

typedef enum { bitDIR, bitINV, bitGET, bitTGL_LAZY, bitTGL, bitSET_LAZY, bitSET, pcf8574BIT_FUNC } pcf8574bit_func_e;

typedef enum { devDIR, devINV, devGET, devTGL_LAZY, devTGL, devSET_LAZY, devSET, pcf8574DEV_FUNC } pcf8574dev_func_e;

// ######################################### Structures ############################################

struct i2c_di_t;
typedef struct __attribute__((packed)) pcf8574_t {
	struct i2c_di_t * psI2C;
	u8_t Mask;						//	all In=0xFF, all Out=0x00
	u8_t Rbuf;
	u8_t Wbuf;
	u8_t fDirty:1;
	u8_t fInvOUT:1;
	u8_t fInvINP:1;
	u8_t spare:5;
} pcf8574_t;
DUMB_STATIC_ASSERT(sizeof(pcf8574_t) == 8);

struct report_t;

// ######################################## Global variables #######################################

// ####################################### Global functions ########################################

/**
 * @brief	Identify device, confirm if PCF8574
 * @param[in]	psI2C pointer to basic I2C device control block
 * @return
 */
int	pcf8574Identify(struct i2c_di_t * psI2C);

/**
 * @brief	Configure PCF8574 device
 * @param[in]	psI2C pointer to basic I2C device control block
 * @return
 */
int	pcf8574Config(struct i2c_di_t * psI2C);

/**
 * @brief	Diagnose PCF8574 device
 * @param[in]	psI2C pointer to basic I2C device control block
 * @return
 */
int	pcf8574Diagnostics(struct i2c_di_t * psI2C);

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

/**
 * @brief	Perform configuration, read, write and toggle operations on device
 * @param[in]	Func enumerated operation function code
 * @param[in]	eChan virtual channel on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	
 */
int pcf8574BitFunction(pcf8574bit_func_e BitFunc, u8_t eChan, bool NewState);

/**
 * @brief	Configure eChan specified as INput
 * @param[in]	eChan on which the operation is to be performed
 * @return	erSUCCESS
 */
#define pcf8574SetDirection(eChan, NewState)	pcf8574BitFunction(bitDIR, eChan, NewState)

/**
 * @brief	Configure eChan specified as INVerted INput
 * @param[in]	eChan on which the operation is to be performed
 * @return	erSUCCESS
 */
#define pcf8574SetInvert(eChan)					pcf8574BitFunction(bitINV, eChan, 0)

/**
 * @brief	Read the state (0/1) of the eChan specified
 * @param[in]	eChan on which the operation is to be performed
 * @return	boolean state of the eChan
 */
#define pcf8574GetState(eChan)					pcf8574BitFunction(bitGET, eChan, 0)

/**
 * @brief	Toggle the state of a eChan (previously configured as OUTput)
 * @param[in]	eChan on which the operation is to be performed
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pcf8574ToggleLazy(eChan)				pcf8574BitFunction(bitTGL_LAZY, eChan, 0)

/**
 * @brief	Toggle the state of a eChan (previously configured as OUTput)
 * @param[in]	eChan on which the operation is to be performed
 * @return	1/true if status written else 0/false
 */
#define pcf8574Toggle(eChan)					pcf8574BitFunction(bitTGL, eChan, 0)

/**
 * @brief	Update buffered pin status without writing new status to device
 * @param[in]	eChan on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pcf8574SetStateLazy(eChan, NewState)	pcf8574BitFunction(bitSET_LAZY, eChan, NewState)

/**
 * @brief	Update buffered pin status and [if changed] write status to device
 * @param[in]	eChan on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if status written else 0/false
 */
#define pcf8574SetState(eChan, NewState)		pcf8574BitFunction(bitSET, eChan, NewState)

int pcf8574DevFunction(pcf8574dev_func_e DevFunc, int Dev, u8_t Mask, bool State);

/**
 * @brief	Configure eChan specified as INput
 * @param[in]	eChan on which the operation is to be performed
 * @return	erSUCCESS
 */
#define pcf8574DevSetDirection(Dev, Mask)			pcf8574DevFunction(devDIR, Dev, Mask, 0)

/**
 * @brief	Configure eChan specified as INVerted INput
 * @param[in]	eChan on which the operation is to be performed
 * @return	erSUCCESS
 */
#define pcf8574DevSetInvert(Dev)					pcf8574DevFunction(devINV, Dev, 0, 0)

/**
 * @brief	Read the state (0/1) of the device specified
 * @param[in]	Dev device on which the operation is to be performed
 * @return	boolean state of the Device
 */
#define pcf8574DevGetState(Dev)						pcf8574DevFunction(devGET, Dev, 0, 0)

/**
 * @brief	Toggle the state of a device (previously configured as OUTput)
 * @param[in]	Dev device on which the operation is to be performed
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pcf8574DevToggleLazy(Dev, Mask)				pcf8574DevFunction(devTGL_LAZY, Dev, Mask, 0)

/**
 * @brief	Toggle the state of a device (previously configured as OUTput)
 * @param[in]	Dev device on which the operation is to be performed
 * @return	1/true if status written else 0/false
 */
#define pcf8574DevToggle(Dev, Mask)					pcf8574DevFunction(devTGL, Dev, Mask, 0)

/**
 * @brief	Update buffered pin status without writing new status to device
 * @param[in]	Dev on which the operation is to be performed
 * @param[in]	NewState optional state for SET type operations
 * @return	1/true if buffer is dirty/changed else 0/false
 */
#define pcf8574DevSetStateLazy(Dev, Mask, State)	pcf8574DevFunction(devSET_LAZY, Dev, Mask, State)

/**
 * @brief	Update buffered pin status and [if changed] write status to device
 * @param[in]	Dev on which the operation is to be performed
 * @param[in]	Mask optional state for SET type operations
 * @return	1/true if status written else 0/false
 */
#define pcf8574DevSetState(Dev, Mask, State)		pcf8574DevFunction(devSET, Dev, Mask, State)

/**
 * @brief
 * @param[in]	psI2C pointer to basic I2C device control block
 * @return
 */
int pcf8574Report(struct report_t * psR);

#ifdef __cplusplus
}
#endif
