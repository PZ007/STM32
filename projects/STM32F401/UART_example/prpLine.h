/**
 *  Filename:       prpLine.h
 *  Platform(s):    STM32F401
 *  Project:        F401_UART_example
 *                  UART example project for STM32F401 microcontrollers
 *  Created:        Dec 19, 2014
 *  Description:    Line data preparation module.
 *                  For demonstration purposes. <br>
 *                  This module reads text characters from a UART and separates
 *                  the lines. <br>
 *                  The '\n' characters are recognized as line separators. <br>
 *
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#ifndef PRPLINE_H_
#define PRPLINE_H_

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
 * Starting the instance twice without stopping it in between returns an error.
 * @return \c R_SUCCESS if the instance hasn't been started already, \c R_ERROR
 * otherwise.
 */
int prpLine_start(void);

/**
 * Stops an instance of this module.
 * Stopping the instance twice without starting it in between returns no error.
 * @return \c R_SUCCESS in any case.
 */
int prpLine_stop(void);

/**
 * Handles the periodic tasks of an instance of this module.
 */
void prpLine_handler(void);

/**
 * Reads a line of text from the console.
 * The returned text is a C string. <br>
 * This function is non-blocking. If no line is available, the returned text is
 * an empty string and the return value is 0.
 *
 * If since the last call of this function a line break has been received,
 * this function returns the characters preceding the line break, followed
 * by a '\n' character followed by a '\0' character. The received characters
 * are thereby removed from the internal console input buffer.
 *
 * Only if a line break has been received, or if the internal buffer is full,
 * a non empty buffer is returned. In the latter case, which indicates a
 * buffer overflow, the last location in the destination buffer contains a '\0'
 * character. No '\n' character is inserted before the last location.
 *
 * If the passed buffer is too small to store the whole line, the line is
 * clipped at its end, and the last location in the buffer contains a '\0'
 * character. No '\n' character is inserted before the last location. The
 * remaining part of the line stays in the internal buffer.
 *
 * In either case just mentioned a missing line break character indicates a
 * buffer overflow condition. The two cases can be distinguished by calling
 * this function a second time. If it returns more characters, the passed
 * destination buffer was too small. If it doesn't, this module's internal
 * buffer was too small to store all data received between two calls to this
 * function. The compile time recovery could be to call this function more
 * frequently. The runtime recovery is to discard the next line (a buffer
 * returned with a '\n''\0' at its end, since the line could be corrupted.)
 *
 * An empty destination buffer contains a '\0' character at index 0.
 * A line break followed by a line break is considered an empty line (not an
 * empty buffer!), in which case a '\n' character followed by a '\0' character
 * is returned.
 *
 * @param[out] dstBuf points to the buffer that will store the line in C string
 * format if this function has completed successfully. In case of an error, it
 * won't be changed.
 * @param[in] dstBufSize is the size of the destination buffer in bytes. It
 * must be greater than zero, otherwise the function will return \c R_ERROR.
 * @return the number of characters in the line, including the '\n' character
 * but excluding the '\0' character, or 0 if no line is available, or
 * \c R_ERROR if an error occurred.
 */
int prpLine_readLine(char* dstBuf, int dstBufSize);

#ifdef __cplusplus
}
#endif

#endif // PRPLINE_H_
