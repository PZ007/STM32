/**
 *  Filename:       init.h
 *  Platform(s):    STM32F401
 *  Project:        F401_UART_example
 *                  UART example project for STM32F401 microcontrollers
 *  Created:        Dec 19, 2014
 *  Description:    Low-level initialization module for startup.
 *  Notes:          
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#ifndef INIT_H_
#define INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Functions
/**
 * Initializes the micro-controller's most basic components.
 * These include the Flash memory controller, the NVIC controller and the
 * clock generation module.
 */
void initBasics(void);

/**
 * Initializes a USART for use by the standard streams, and therefore
 * subsequently by the printf() and scanf() functions, to mention two of the
 * standard C library console I/O functions. <br>
 * This function initializes the USART defined by the symbol CFG_CONSOLE_USART.
 * This symbol is usually defined in cfgProject.h. Its value is typically
 * USART1 (a pointer to a USART base address), but could also point to any other
 * USART base address. If the symbol is undefined, the implementation of this
 * function is suppressed by a conditional compilation clause. As long as it
 * isn't called, there is no problem. If it's yet called, the linker will
 * report an error.
 */
void initConsoleUSART(void);

#ifdef __cplusplus
}
#endif

#endif // INIT_H_
