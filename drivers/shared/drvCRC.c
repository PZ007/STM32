/**
 *  Filename:       drvCRC.c
 *  Platform(s):    STM32F100, STM32F401
 *  Project:
 *  Created:        Nov 17, 2014
 *  Description:    Cyclic Redundancy Check driver module.
 *
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#include "drvCRC.h"

#include <stdio.h>
#include <string.h>

#ifdef STM32F10X_MD_VL
#include <stm32f10x.h>
#else
#ifdef STM32F401xx
#include <stm32f4xx.h>
#else
#error // Enforce definition of preprocessor symbol
#endif
#endif

#include <cfgGlobal.h>
#include <cfgPlatform.h>

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types
/** Instance descriptor type. */
typedef struct inst_struct
{
    /** \c TRUE if instance started, \c FALSE otherwise. */
    BOOL started;
} inst_t;

//------------------------------------------------------------------------------
// Constants and Variables
static inst_t sInstDscr; /**< Instance descriptor. */

//------------------------------------------------------------------------------
// Local Functions
/**
 * Initializes the variables of the module instance.
 * @param[in] instId is the instance identifier.
 */
static void initInst(void)
{
    memset(&sInstDscr, 0, sizeof(sInstDscr));
}

//------------------------------------------------------------------------------
// Global Functions
int drvCRC_start(void)
{
    CHECK_NOT_STARTED_INT(&sInstDscr, 0);

    initInst();

    /* Enable the clock to the SoC device. */
#ifdef STM32F10X_MD_VL
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
#else
#ifdef STM32F401xx
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
#else
#error
#endif
#endif

    /* Reset the SoC device via its own reset bit. */
    CRC->CR |= CRC_CR_RESET;

    sInstDscr.started = TRUE;
    return R_SUCCESS;
}

int drvCRC_stop(void)
{
    if (sInstDscr.started)
    {
#ifdef STM32F10X_MD_VL
        /* Disable the clock to the SoC device. */
        RCC->AHBENR &= ~RCC_AHBENR_CRCEN;
#else
#ifdef STM32F401xx
        /* Reset the SoC device via the RCC unit. */
        RCC->AHB1RSTR |= RCC_AHB1RSTR_CRCRST;
        while ((RCC->AHB1RSTR & RCC_AHB1RSTR_CRCRST) == 0); // Necessary?
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CRCRST;

        /* Disable the clock to the SoC device. */
        RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
#else
#error
#endif
#endif

        sInstDscr.started = FALSE;
    }
    return R_SUCCESS;
}

int drvCRC_calculate(const void* data, int dataSize, int* result)
{
    CHECK_STARTED_INT(&sInstDscr, 0);
    CHECK_POINTER_INT(data);
    CHECK_POINTER_INT(result);

    const int* pInt = data; // For simple type casting from void* to int*

    if (dataSize % sizeof(int) == 0)
    {
        *result = 0;
        while (dataSize > 0)
        {
            CRC->DR = *pInt++;
            dataSize -= sizeof(int);
        }
        *result = CRC->DR;
        return R_SUCCESS;
    }
    else
    {
        PRINT_ERROR("Arg dataSize isn't modulo of %i: %i", sizeof(int), dataSize);
    }

    return R_ERROR;
}

int drvCRC_reset(void)
{
    CHECK_STARTED_INT(&sInstDscr, 0);

    CRC->CR |= CRC_CR_RESET;

    return R_SUCCESS;
}
