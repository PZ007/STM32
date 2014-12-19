/**
 *  Filename:       retarget.c
 *  Platform(s):    STM32F401
 *  Project:        F401_UART_example
 *                  UART example project for STM32F401 microcontrollers
 *  Created:        Dec 19, 2014
 *  Description:    Retarget low-level C library functions module.
 *                  
 *                  Implements target dependent low-level functions required
 *                  by the C library.
 *
 *                  If this module is included in a project, the symbol
 *                  CFG_CONSOLE_USART has to be defined somewhere in the
 *                  project, usually in cfgProject.h. Its value is typically
 *                  USART1 (a pointer to a USART base address), but could also
 *                  point to any other USART base address.
 *
 *  Notes:          The public functions aren't exported by a retarget.h file.
 *                  They are exported by library and system header files.
 *
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#include <stdio.h>

#include <stm32f4xx.h>

#include "cfgProject.h"

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Local Functions
static int readChar(void)
{
    while(!(CFG_CONSOLE_USART->SR & 0x0020));
    return (CFG_CONSOLE_USART->DR);
}

static int writeChar(int c)
{
    while(!(CFG_CONSOLE_USART->SR & 0x0080));
    CFG_CONSOLE_USART->DR = (c & 0x1FF);
    return c;
}

//------------------------------------------------------------------------------
// Global Functions
/**
 * Get character.
 * Low-level library function. Implements the target dependent part of the C
 * library. <br>
 * Blocks until one character could be read from the lower level driver.
 * @param[in] f points to the file descriptor. Required for compatibility with
 * the C library. Unused.
 * @return read character promoted to an \c int value.
 */
int fgetc(FILE *f)
{
    return (readChar());
}

/**
 * Put character.
 * Low-level library function. Implements the target dependent part of the C
 * library. <br>
 * Blocks until the character could be written to the lower level driver.
 * @param[in] ch is the \c int promotion of the character to be written. The
 * value is internally converted to an <tt> unsigned char </tt> when written.
 * @param[in] f points to the file descriptor. Required for compatibility with
 * the C library. Unused.
 * @return the character written.
 */
int fputc(int ch, FILE *f)
{
    return (writeChar(ch));
}

/**
 * System exit.
 * Low-level library function. Implements the target dependent part of the C
 * library. <br>
 * Blocks forever.
 * param[in] return_code is the return value passed from the \c main()
 * function. Required for compatibility with the C library. Unused.
 */
void _sys_exit(int return_code)
{
    label: goto label;
}
