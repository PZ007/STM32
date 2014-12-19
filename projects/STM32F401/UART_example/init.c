/**
 *  Filename:       init.c
 *  Platform(s):    STM32F401
 *  Project:        F401_UART_example
 *                  UART example project for STM32F401 microcontrollers
 *  Created:        Dec 19, 2014
 *  Description:    Low-level initialization module for startup.
 *
 *                  Implements low-level initialization functions of controller
 *                  modules required by the application.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#include "init.h"

#include <misc.h>
#include <stm32f4xx.h>

#include <cfgPlatform.h>
#include "cfgProject.h"

//------------------------------------------------------------------------------
// Symbols and Macros
/* Copied from Keil's STM32F401 Blinky project, file Serial.c; parentheses
 * added around arguments.
 */
#define __DIV(__PCLK, __BAUD) \
    ((__PCLK*25)/(4*__BAUD))

#define __DIVMANT(__PCLK, __BAUD) \
    (__DIV((__PCLK), (__BAUD))/100)

#define __DIVFRAQ(__PCLK, __BAUD) \
    (((__DIV((__PCLK), (__BAUD)) - \
    (__DIVMANT((__PCLK), (__BAUD)) * 100))*16+50) / 100)

#define __USART_BRR(__PCLK, __BAUD) \
    ((__DIVMANT((__PCLK), (__BAUD)) << 4)|(__DIVFRAQ((__PCLK), (__BAUD)) & 0x0F))

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Local Functions

//------------------------------------------------------------------------------
// Global Functions
void initBasics(void)
{
    /* Enable data and instruction cache and prefetch buffer, set wait states
     * for Flash memory access.
     */
    FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN
        | FLASH_ACR_LATENCY_1WS;

    /* Set 4 bits for group (equal to preemption) priority and 0 bits for
     * subpriority. Arbitrary choice. Project dependent.
     * Note: use this function provided by core_cm4.h instead of
     * NVIC_PriorityGroupConfig() implemented in misc.c of the SPL because
     * it's safer: the latter only sets the bits using bitwise OR, the former
     * clears the bits first before setting them.
     */
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);

    /* Select HSI as main oscillator. The HSE oscillator isn't populated on
     * the Nucleo board.
     */
    RCC->CR |= ((uint32_t)RCC_CR_HSION); // Enable HSI
    while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI ready

    RCC->CFGR = RCC_CFGR_SW_HSI; // Set HSI as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    // Wait for HSI used as system clock

    /* Configure the HCLK, APB1 and APB2 clock frequency. */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // HCLK = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // APB1 = HCLK/1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = HCLK/1

    /* Configure and enable the PLL. */
    RCC->CR &= ~RCC_CR_PLLON; // Disable the PLL

    /* PLL configuration: run the CPU at 32MHz;
     * f_VCO = f_HSI*(N/M), f_PLL_out = f_VCO/P
     */
    RCC->PLLCFGR =
        ( 16ul                   | // PLL_M =  16
        (192ul <<  6)            | // PLL_N = 192
        (  2ul << 16)            | // PLL_P =   6
        (RCC_PLLCFGR_PLLSRC_HSI) | // PLL_SRC = HSI
        (  4ul << 24)             ); // PLL_Q = 4

    RCC->CR |= RCC_CR_PLLON; // Enable the PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP(); // Wait until PLL ready

    /* Select the PLL as system clock source. */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    // Wait until PLL is system clock source

    /* Update the global SystemCoreClock variable that stores the core
     * clock frequency in Hertz.
     */
    SystemCoreClockUpdate();
}

/* This #ifdef condition allows to leave CFG_CONSOLE_USART undefined and
 * yet use initBasics() (and therefore adding this file to the project)
 * without causing the compiler to report the error 'identifier
 * "CFG_CONSOLE_USART" is undefined'.
 */
#ifdef CFG_CONSOLE_USART

/* Copied from Keil's STM32F401 Blinky project, file Serial.c, and renamed.
 * USART1 added.
 */
void initConsoleUSART(void)
{
    unsigned int APB_clkFreq;

    RCC->AHB1ENR  |=   ( 1ul <<  0); // Enable GPIOA clock
    if (CFG_CONSOLE_USART == USART1)
    {
        RCC->APB2ENR  |=   ( 1ul <<  4); // Enable USART#1 clock
        APB_clkFreq = CFG_GET_APB2_CLK();
        /* Configure PA10 to USART1_RX, PA9 to USART1_TX */
        GPIOA->AFR[1] &= ~((15ul << 4* 2) | (15ul << 4* 1) );
        GPIOA->AFR[1] |=  (( 7ul << 4* 2) | ( 7ul << 4* 1) );
        GPIOA->MODER  &= ~(( 3ul << 2*10) | ( 3ul << 2* 9) );
        GPIOA->MODER  |=  (( 2ul << 2*10) | ( 2ul << 2* 9) );
    }
    else if (CFG_CONSOLE_USART == USART2)
    {
        RCC->APB1ENR  |=   ( 1ul << 17); // Enable USART#2 clock
        APB_clkFreq = CFG_GET_APB1_CLK();
        /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
        GPIOA->AFR[0] &= ~((15ul << 4* 3) | (15ul << 4* 2) );
        GPIOA->AFR[0] |=  (( 7ul << 4* 3) | ( 7ul << 4* 2) );
        GPIOA->MODER  &= ~(( 3ul << 2* 3) | ( 3ul << 2* 2) );
        GPIOA->MODER  |=  (( 2ul << 2* 3) | ( 2ul << 2* 2) );
    }
    else
    {
        while(1); // Initialization not implemented. Error. Fail early.
    }

    CFG_CONSOLE_USART->BRR  = __USART_BRR(APB_clkFreq, 115200);
    CFG_CONSOLE_USART->CR3  = 0x0000;            // No flow control
    CFG_CONSOLE_USART->CR2  = 0x0000;            // 1 stop bit
    CFG_CONSOLE_USART->CR1  = ((   1ul <<  2) |  // Enable RX
                               (   1ul <<  3) |  // Enable TX
                               (   0ul << 12) |  // 1 start bit, 8 data bits
                               (   1ul << 13) ); // Enable USART
}

#endif
