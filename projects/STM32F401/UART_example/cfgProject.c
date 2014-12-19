/**
 *  Filename:       cfgProject.c
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

#include "cfgProject.h"

#include <stm32f4xx.h>

#include "drvGPIO.h"
#include "drvUART.h"

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables
/** Receive buffers for the UART driver instances. */
static char sDrvUART_rxBufInstN0[CFG_DRVUART_RX_BUF_SIZE_N0];
static char sDrvUART_rxBufInstN1[CFG_DRVUART_RX_BUF_SIZE_N1];
static char sDrvUART_rxBufInstN2[CFG_DRVUART_RX_BUF_SIZE_N2];

/** Transmit buffers for the UART driver instances. */
static char sDrvUART_txBufInstN0[CFG_DRVUART_TX_BUF_SIZE_N0];
static char sDrvUART_txBufInstN1[CFG_DRVUART_TX_BUF_SIZE_N1];
static char sDrvUART_txBufInstN2[CFG_DRVUART_TX_BUF_SIZE_N2];

const drvUART_cfg_t cfgDrvUART[] =
{
    // instId = 0
    {
        7,                              // alternateFunctionSelection
        10,                             // GPIO_portBitNumberRx
        9,                              // GPIO_portBitNumberTx
        DRV_GPIO_INST_A,                // instIdGPIO
        USART1_IRQn,                    // NVIC_IRQ_channel
        CFG_DRVUART_IRQ_GROUP_PRIO_N0,  // NVIC_IRQ_groupPriority
        CFG_DRVUART_IRQ_SUB_PRIO_N0,    // NVIC_IRQ_subPriority
        RCC_APB2ENR_USART1EN,           // RCC_clockEnableBitMask
        &(RCC->APB2ENR),                // RCC_clockEnableRegister
        USART1,                         // USARTx
        sDrvUART_rxBufInstN0,           // rxBuffer
        CFG_DRVUART_RX_BUF_SIZE_N0,     // rxBufferSize
        sDrvUART_txBufInstN0,           // txBuffer
        CFG_DRVUART_TX_BUF_SIZE_N0,     // txBufferSize
    },
    // instId = 1
    {
        7,                              // alternateFunctionSelection
        3,                              // GPIO_portBitNumberRx
        2,                              // GPIO_portBitNumberTx
        DRV_GPIO_INST_A,                // instIdGPIO
        USART2_IRQn,                    // NVIC_IRQ_channel
        CFG_DRVUART_IRQ_GROUP_PRIO_N1,  // NVIC_IRQ_groupPriority
        CFG_DRVUART_IRQ_SUB_PRIO_N1,    // NVIC_IRQ_subPriority
        RCC_APB1ENR_USART2EN,           // RCC_clockEnableBitMask
        &(RCC->APB1ENR),                // RCC_clockEnableRegister
        USART2,                         // USARTx
        sDrvUART_rxBufInstN1,           // rxBuffer
        CFG_DRVUART_RX_BUF_SIZE_N1,     // rxBufferSize
        sDrvUART_txBufInstN1,           // txBuffer
        CFG_DRVUART_TX_BUF_SIZE_N1,     // txBufferSize
    },
    // instId = 2
    {
        8,                              // alternateFunctionSelection
        12,                             // GPIO_portBitNumberRx
        11,                             // GPIO_portBitNumberTx
        DRV_GPIO_INST_A,                // instIdGPIO
        USART6_IRQn,                    // NVIC_IRQ_channel
        CFG_DRVUART_IRQ_GROUP_PRIO_N2,  // NVIC_IRQ_groupPriority
        CFG_DRVUART_IRQ_SUB_PRIO_N2,    // NVIC_IRQ_subPriority
        RCC_APB2ENR_USART6EN,           // RCC_clockEnableBitMask
        &(RCC->APB2ENR),                // RCC_clockEnableRegister
        USART6,                         // USARTx
        sDrvUART_rxBufInstN2,           // rxBuffer
        CFG_DRVUART_RX_BUF_SIZE_N2,     // rxBufferSize
        sDrvUART_txBufInstN2,           // txBuffer
        CFG_DRVUART_TX_BUF_SIZE_N2,     // txBufferSize
    },
};


//------------------------------------------------------------------------------
// Local Functions

//------------------------------------------------------------------------------
// Global Functions
