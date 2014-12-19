/**
 *  Filename:       cfgPlatform.h
 *  Platform(s):    STM32F401
 *  Project:
 *  Created:        Mar 19, 2014
 *  Description:    Platform configuration module.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#ifndef CFGPLATFORM_H_
#define CFGPLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include <stm32f4xx.h>

//------------------------------------------------------------------------------
// Symbols and Macros
/**
 * Disable all interrupts with configurable priority.
 * Only the interrupts \c Reset, \c NMI and <tt>Hard fault</tt> have got non-
 * configurable priorities. Thus, all other are disabled. <br>
 * This is Cortex M4 specific. See ST Programming Manual PM0214, CMSIS
 * intrinsic functions.
 */
#define DISABLE_INTERRUPTS()            __disable_irq() // Intrinsic CMSIS call

/**
 * Enable all interrupts with configurable priority.
 * Only the interrupts \c Reset, \c NMI and <tt>Hard fault</tt> have got non-
 * configurable priorities. <br>
 * This is Cortex M4 specific. See ST Programming Manual PM0214, CMSIS
 * intrinsic functions.
 */
#define ENABLE_INTERRUPTS()             __enable_irq()  // Intrinsic CMSIS call

/**
 * Returns the bit position of the first non-zero bit from the LSB.
 * Depending on the version of stm32f4xx.h, it is already defined there.
 */
#ifndef POSITION_VAL
#define POSITION_VAL(VAL)               (__CLZ(__RBIT(VAL)))
#endif

/*
 * If the PRINT_ macros haven't been defined on project level (inside
 * cfgProject.h), then they are defined here.
 */
#ifdef __DEBUG

#ifndef PRINT_DEBUG
#define PRINT_DEBUG(...) \
    printf("+++ DEBUG +++: Func %s, line %i:\n", \
        __func__, __LINE__); \
    printf(__VA_ARGS__); \
    printf("\n");
#endif

#ifndef PRINT_ERROR
#define PRINT_ERROR(...) \
    printf("+++ ERROR +++: Func %s, line %i:\n", \
        __func__, __LINE__); \
    printf(__VA_ARGS__); \
    printf("\n");
#endif

#ifndef PRINT_WARNG
#define PRINT_WARNG(...) \
    printf("+++ WARNG +++: Func %s, line %i:\n", \
        __func__, __LINE__); \
    printf(__VA_ARGS__); \
    printf("\n");
#endif

#else // __DEBUG symbol not defined

#ifndef PRINT_DEBUG
#define PRINT_DEBUG(...)
#endif

#ifndef PRINT_ERROR
#define PRINT_ERROR(...)
#endif

#ifndef PRINT_WARNG
#define PRINT_WARNG(...)
#endif

#endif

/** Returns the APB1 clock frequency in Hertz. */
#define CFG_GET_APB1_CLK() \
    (SystemCoreClock >> cfgAPB_AHB_prescalerTable[ \
        (RCC->CFGR & RCC_CFGR_PPRE1) >> POSITION_VAL(RCC_CFGR_PPRE1)])

/** Returns the APB2 clock frequency in Hertz. */
#define CFG_GET_APB2_CLK() \
    (SystemCoreClock >> cfgAPB_AHB_prescalerTable[ \
        (RCC->CFGR & RCC_CFGR_PPRE2) >> POSITION_VAL(RCC_CFGR_PPRE2)])

//------------------------------------------------------------------------------
// Types
/** Time structure as defined also in <sys/time.h> of C libraries. */
struct timeval
{
    unsigned int tv_sec;
    unsigned int tv_usec;
};

//------------------------------------------------------------------------------
// Constants and Variables
/* Copied from stm32f4xx_rcc.c. Is identical to the one in stm32f4xx_hal_rcc.c.
 * Added here to break dependencies to STM's HAL library.
 */
extern const unsigned char cfgAPB_AHB_prescalerTable[];

//------------------------------------------------------------------------------
// Functions

#ifdef __cplusplus
}
#endif

#endif /* CFGPLATFORM_H_ */
