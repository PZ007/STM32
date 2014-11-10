/**
 *  Filename:       drvGPIO.h
 *  Platform(s):    STM32F10x
 *  Project:
 *  Created:        Feb 11, 2014
 *  Description:    General Purpose Inputs and Outputs driver module.
 *
 *                  The STM32F10x devices have got three to five GPIO ports.
 *                  This module implements a multi-instance driver for these
 *                  ports, introducing one instance per port. The assignment
 *                  of an instance number to a port is defined by the
 *                  enumeration \c drvGPIO_inst_t.
 *
 *  Notes:          The five instances are statically allocated. Thus the RAM
 *                  required by this driver is fix.
 *
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#ifndef DRVGPIO_H_
#define DRVGPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// Symbols and Macros
/** Instance identifiers and count of GPIO driver. */
typedef enum drvGPIO_inst_enum
{
    DRV_GPIO_INST_A,
    DRV_GPIO_INST_B,
    DRV_GPIO_INST_C,
    DRV_GPIO_INST_D,
    DRV_GPIO_INST_E,
    DRV_GPIO_INST_CNT,
} drvGPIO_inst_t;

/** Input/output mode enumeration. */
typedef enum drvGPIO_IO_mode_enum
{
    DRV_GPIO_IO_MODE_AF_OUT_OPEN_DRAIN,
    DRV_GPIO_IO_MODE_AF_OUT_PUSH_PULL,
    DRV_GPIO_IO_MODE_IN_ANALOG,
    DRV_GPIO_IO_MODE_IN_FLOATING,
    DRV_GPIO_IO_MODE_IN_PULL_DOWN,
    DRV_GPIO_IO_MODE_IN_PULL_UP,
    DRV_GPIO_IO_MODE_OUT_OPEN_DRAIN,
    DRV_GPIO_IO_MODE_OUT_PUSH_PULL,
    DRV_GPIO_IO_MODE_CNT,
} drvGPIO_IO_mode_t;

/** Maximum output frequency enumeration. */
typedef enum drvGPIO_outMaxFreq_enum
{
    DRV_GPIO_OUT_MAX_FREQ_10MHZ = 1,
    DRV_GPIO_OUT_MAX_FREQ_2MHZ,
    DRV_GPIO_OUT_MAX_FREQ_50MHZ,
    DRV_GPIO_OUT_MAX_FREQ_CNT,
} drvGPIO_outMaxFreq_t;

//------------------------------------------------------------------------------
// Types
/**
 * Port pin attribute structure.
 * Attributes are the I/O mode and output speed.
 * This structure also contains a member that specifies the port bits to
 * which these attributes shall be applied.
 */
typedef struct drvGPIO_pinAttr_struct
{
    /**
     * Specifies the port pins to be configured.
     * All bits containing the value 1 indicate that the corresponding port pin
     * shall be configured.
     */
    short bitPattern;

    /** Output maximum frequency. */
    drvGPIO_outMaxFreq_t outMaxFreq;
    
    /** Input/output mode of the port bit. */
    drvGPIO_IO_mode_t IO_mode;
} drvGPIO_pinAttr_t;

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Functions
/**
 * Starts an instance (port) of this module.
 * The GPIO clock is enabled. <br>
 * Starting an instance twice without stopping it in between returns an
 * error.
 * @param[in] instId is the instance identifier.
 * @return \c R_SUCCESS if the instance hasn't been started already, \c R_ERROR
 * otherwise.
 */
int drvGPIO_start(int instId);

/**
 * Stops an instance (port) of this module.
 * This function doesn't change the port configuration nor does it disable
 * the GPIO clock. It could be in use by another peripheral module. <br>
 * Stopping an instance twice without starting it in between returns no
 * error.
 * @param[in] instId is the instance identifier.
 * @return \c R_SUCCESS if the parameter checks pass, \c R_ERROR otherwise.
 */
int drvGPIO_stop(int instId);

/**
 * Clears 0 to 16 bits of the selected port to 0.
 * @param[in] instId is the instance identifier.
 * @param[in] bitPattern specifies the bits to be cleared. All bits containing
 * the value 1 will be cleared using an atomic instruction.
 * @return \c R_SUCCESS if the parameter checks pass and the instance has been
 * started, \c R_ERROR otherwise.
 */
int drvGPIO_clrOutputBits(int instId, unsigned short bitPattern);

/**
 * Configures one to sixteen I/O pins of the selected port.
 * @param[in] instId is the instance identifier.
 * @param[in] pinAttr points to the configuration structure that specifies
 * what I/O pins are to be configured and how. Members unused for the selected
 * configuration are ignored. However, they still mustn't exceed their valid 
 * range, or an error will be returned.
 * @return \c R_SUCCESS if the parameter checks pass and the instance has been
 * started, \c R_ERROR otherwise.
 */
int drvGPIO_configPin(int instId, const drvGPIO_pinAttr_t* pinAttr);

/** Returns the status of the 16 I/Os of the selected port.
 * @param[in] instId is the instance identifier.
 * @param[out] value points to the variable that will hold the value of the
 * sixteen port bits after this function has returned successfully. In case of
 * an error it won't be changed.
 * @return \c R_SUCCESS if the parameter checks pass and the instance has been
 * started, \c R_ERROR otherwise.
 */
int drvGPIO_readIO_state(int instId, unsigned short* value);

/**
 * Sets 0 to 16 bits of the selected port to 1.
 * @param[in] instId is the instance identifier.
 * @param[in] bitPattern specifies the bits to be set. All bits containing
 * the value 1 will be set using an atomic instruction.
 * @return \c R_SUCCESS if the parameter checks pass and the instance has been
 * started, \c R_ERROR otherwise.
 */
int drvGPIO_setOutputBits(int instId, unsigned short bitPattern);

#ifdef __cplusplus
}
#endif

#endif // DRVGPIO_H_
