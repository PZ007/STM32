/**
 *  Filename:       drvCRC.h
 *  Platform(s):    STM32F100, STM32F401
 *  Project:
 *  Created:        Nov 17, 2014
 *  Description:    Cyclic Redundancy Check driver module.
 *
 *                  The STM32F1/2/3/4 devices have got an internal CRC
 *                  calculation unit. This module implements a single-instance
 *                  driver for it.
 *
 *                  \c drvCRC_calculate() calculates the 32bit CRC-32 checksum
 *                  with the generator polynomial value (divisor) 0x04C11DB7
 *                  and the start value 0xFFFFFFFF.
 *  Notes:
 *
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#ifndef DRVCRC_H_
#define DRVCRC_H_

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Functions
/**
 * Starts an instance of this module.
 * Starting an instance twice without stopping it in between returns an error.
 * @return \c R_SUCCESS if the instance hasn't been started already, \c R_ERROR
 * otherwise.
 */
int drvCRC_start(void);

/**
 * Stops an instance of this module.
 * Stopping an instance twice without starting it in between returns no error.
 * @return \c R_SUCCESS in any case.
 */
int drvCRC_stop(void);

/**
 * Calculates the 32bit checksum of the data in the passed buffer.
 * The type of the checksum is called CRC-32. This defines not only its size in
 * bits, but also its generator polynomial value. <br>
 * Generator polynomial names: CRC-32 <br>
 * Generator polynomial value: 0x04C11DB7 <br>
 * Start value: 0xFFFFFFFF <br>
 * @param[in] data points to the data for which the checksum shall be
 * calculated.
 * @param[in] dataSize is the size of the data in bytes.
 * @param[out] points to the number that will hold the checksum after this
 * function has returned successfully. In case of an error, it won't be
 * changed.
 * @return \c R_SUCCESS if the parameter checks pass, \c R_ERROR otherwise.
 */
int drvCRC_calculate(const void* data, int dataSize, int* result);

/**
 * Resets the CRC calculation unit.
 * Calling this function sets the internal data register to 0xFFFFFFFF, which
 * is the start value.
 * @return \c R_SUCCESS if the instance has been started, \c R_ERROR otherwise.
 */
int drvCRC_reset(void);

#ifdef __cplusplus
}
#endif

#endif // DRVCRC_H_
