/**
 *  Filename:       cfgProject.h
 *  Platform(s):    STM32F401
 *  Project:        F401_UART_example
 *                  UART example project for STM32F401 microcontrollers
 *  Created:        Dec 19, 2014
 *  Description:    
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#ifndef CFGPROJECT_H_
#define CFGPROJECT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>

#include <cfgPlatform.h>

#include "drvUART.h"

//------------------------------------------------------------------------------
// Symbols and Macros
/** Selects the USART used by the C library for the standard streams stdin,
 * stdout and stderr.
 */
#define CFG_CONSOLE_USART               USART1

/** NVIC IRQ group priority of UART driver instance 0. */
#define CFG_DRVUART_IRQ_GROUP_PRIO_N0   3
/** NVIC IRQ group priority of UART driver instance 1. */
#define CFG_DRVUART_IRQ_GROUP_PRIO_N1   3
/** NVIC IRQ group priority of UART driver instance 2. */
#define CFG_DRVUART_IRQ_GROUP_PRIO_N2   3

/** NVIC IRQ sub-priority of UART driver instance 0. */
#define CFG_DRVUART_IRQ_SUB_PRIO_N0     3
/** NVIC IRQ sub-priority of UART driver instance 1. */
#define CFG_DRVUART_IRQ_SUB_PRIO_N1     3
/** NVIC IRQ sub-priority of UART driver instance 2. */
#define CFG_DRVUART_IRQ_SUB_PRIO_N2     3

/** Size of UART driver instance 0 receive buffer in bytes. */
#define CFG_DRVUART_RX_BUF_SIZE_N0      16
/** Size of UART driver instance 1 receive buffer in bytes. */
#define CFG_DRVUART_RX_BUF_SIZE_N1      128
/** Size of UART driver instance 2 receive buffer in bytes. */
#define CFG_DRVUART_RX_BUF_SIZE_N2      16

/** Size of UART driver instance 0 transmit buffer in bytes. */
#define CFG_DRVUART_TX_BUF_SIZE_N0      16
/** Size of UART driver instance 1 transmit buffer in bytes. */
#define CFG_DRVUART_TX_BUF_SIZE_N1      128
/** Size of UART driver instance 2 transmit buffer in bytes. */
#define CFG_DRVUART_TX_BUF_SIZE_N2      16

/*
 * If project specific implementations of the PRINT_... macros are required,
 * define them here. If not, they have got a default implementation in
 * \c cfgPlatform.h.
 */
// #define PRINT_DEBUG(...)                printf(__VA_ARGS__)
// #define PRINT_ERROR(...)                printf(__VA_ARGS__)
// #define PRINT_WARNG(...)                printf(__VA_ARGS__)

/** Time of the long application cycle in milliseconds. */
#define CFG_CYCLE_PERIOD_LONG           (10 * CFG_CYCLE_PERIOD_SHORT)

/**
  * Time of the short application cycle in milliseconds.
  * Dependency of other modules for precise time calculations:
  * 1000 divided by this number should result in an integer number (remainder
  * equals to zero).
  */
#define CFG_CYCLE_PERIOD_SHORT          1

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables
/** Instance identifiers and count of UART driver. */
enum
{
    CFG_DRVUART_INST_N0,
    CFG_DRVUART_INST_N1,
    CFG_DRVUART_INST_N2,
    CFG_DRVUART_INST_CNT,
};

extern const drvUART_cfg_t cfgDrvUART[CFG_DRVUART_INST_CNT];

//------------------------------------------------------------------------------
// Functions

#ifdef __cplusplus
}
#endif

#endif // CFGPROJECT_H_
