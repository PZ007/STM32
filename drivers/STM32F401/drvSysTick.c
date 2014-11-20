/**
 *  Filename:       drvSysTick.c
 *  Platform(s):    STM32F401
 *  Project:
 *  Created:        Mar 20, 2014
 *  Description:    System Tick driver module.
 *
 *  Notes:          Uses the STM32F401 internal System Tick Timer as timer base.
 *                  Comments on timer configuration:
 *                  - The timer uses the processor clock (AHB) as clock source.
 *                    Since the counter registers only use 24bits out of 32bits,
 *                    the maximum tick time is 699ms at 24MHz clock frequency.
 *                    This configuration is set by calling the provided
 *                    SysTick_Config() function.
 *                  - The timer could be configured to use the processor clock
 *                    divided by 8. This would result in an 8 times longer
 *                    maximum timer value, at the cost of the re-implementation
 *                    of SysTick_Config() in this module.
 *
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#include "drvSysTick.h"

#include <stdio.h>
#include <string.h>

#include <core_cm4.h> // For SysTick_Config(), SysTick->...
#include <system_stm32f4xx.h> // For SystemCoreClock

#include <cfgGlobal.h>
#include <cfgPlatform.h>

//------------------------------------------------------------------------------
// Symbols and Macros
/** SysTick Control and Status register: state of the bits after a reset. */
#define STK_CTRL_RESET_BIT_MASK         0x04

//------------------------------------------------------------------------------
// Types
/** Instance descriptor type. */
typedef struct inst_struct
{
    /** TRUE if the timer expired, FALSE otherwise. */
    volatile BOOL expired;

    /** Number of ticks missed since the start of this instance. */
    unsigned long long missed;

    /** Period of the timer in microseconds. */
    unsigned int period_us;

    /** \c TRUE if instance started, \c FALSE otherwise. */
    BOOL started;

    /**
     * Time passed since the start of this instance.
     * The resolution of this variable is limited to the resolution of the
     * timer.
     */
    struct timeval uptime;
} inst_t;

//------------------------------------------------------------------------------
// Constants and Variables
static inst_t sInstDscr; /**< Instance descriptor. */

//------------------------------------------------------------------------------
// Local Functions
/**
 * Initializes the variables of the driver instance.
 */
static void initInst(void)
{
    memset(&sInstDscr, 0, sizeof(sInstDscr));
}

//------------------------------------------------------------------------------
// Global Functions
/**
 * Interrupt handler of the System Tick Timer.
 * This function is declared in the file startup_stm32f401xe.s.
 */
void SysTick_Handler(void)
{
    sInstDscr.uptime.tv_usec += sInstDscr.period_us;
    sInstDscr.uptime.tv_sec += sInstDscr.uptime.tv_usec / 1000000;
    sInstDscr.uptime.tv_usec %= 1000000;

    if (sInstDscr.expired)
    {
        sInstDscr.missed += 1;
    }
    else
    {
        sInstDscr.expired = TRUE;
    }
}

int drvSysTick_start(unsigned int period_us)
{
    unsigned int ticks;

    CHECK_NOT_STARTED_INT(&sInstDscr, 0);

    if (period_us >= DRV_SYSTICK_PERIOD_US_MIN &&
        period_us <= DRV_SYSTICK_PERIOD_US_MAX)
    {
        initInst();

        ticks = (unsigned int)
            (((unsigned long long)period_us * SystemCoreClock) / 1000000);
        if (SysTick_Config(ticks) == 0)
        {
            sInstDscr.period_us = period_us;
            sInstDscr.started = TRUE;
            return R_SUCCESS;
        }
        else
        {
            PRINT_ERROR("Call to SysTick_Config() failed");
        }
    }
    else
    {
        PRINT_ERROR("Param period_us outside range: %u", period_us);
    }
    return R_ERROR;
}

int drvSysTick_stop(void)
{
    if (sInstDscr.started)
    {
        SysTick->CTRL = STK_CTRL_RESET_BIT_MASK;
        sInstDscr.started = FALSE;
    }
    return R_SUCCESS;
}

int drvSysTick_getMissed(unsigned long long* missed)
{
    CHECK_STARTED_INT(&sInstDscr, 0);
    CHECK_POINTER_INT(missed);

    *missed = sInstDscr.missed;
    return R_SUCCESS;
}

int drvSysTick_getPeriod(unsigned int* period_us)
{
    CHECK_STARTED_INT(&sInstDscr, 0);
    CHECK_POINTER_INT(period_us);

    *period_us = sInstDscr.period_us;
    return R_SUCCESS;
}

int drvSysTick_getUptime(struct timeval* uptime)
{
    unsigned int sysTickValue; // System Tick counter counts down!

    CHECK_STARTED_INT(&sInstDscr, 0);
    CHECK_POINTER_INT(uptime);

    /* Note that SysTick->VAL counts down and rolls over to SysTick->LOAD
     * when reaching 0 even if interrupts are disabled.
     * Therefore, if SysTick->VAL rolls over while reading out uptime,
     * try again. Could use outdated uptime otherwise.
     */
    do
    {
        SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
        DISABLE_INTERRUPTS();
        sysTickValue = SysTick->VAL;
        *uptime = sInstDscr.uptime;
        ENABLE_INTERRUPTS();
    } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
    sysTickValue = SysTick->LOAD - sysTickValue;
    /* Initial code, when uptime was of type unsigned long long:
     * *uptime += (unsigned long long)sysTickValue * 1000000 / SystemCoreClock;
     * However, this invokes the function __aeabi_uldivmod() for the 64bit
     * integer division. Takes quite long. With a simple trick (divide
     * numerator and divisor by 1000) the division can be kept in the integer
     * range. That causes the compiler to use the UDIV instruction, which is
     * much faster executed than the __aeabi_uldivmod() function.
     */
    uptime->tv_usec += sysTickValue * 1000UL / (SystemCoreClock / 1000UL);
    uptime->tv_sec += uptime->tv_usec / 1000000UL;
    uptime->tv_usec %= 1000000UL;
    return R_SUCCESS;
}

int drvSysTick_waitForTick(void)
{
    CHECK_STARTED_INT(&sInstDscr, 0);

    while (!sInstDscr.expired);
    sInstDscr.expired = FALSE;
    return R_SUCCESS;
}
