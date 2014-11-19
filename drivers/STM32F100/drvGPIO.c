/**
 *  Filename:       drvGPIO.c
 *  Platform(s):    STM32F10x
 *  Project:
 *  Created:        Feb 11, 2014
 *  Description:    General Purpose Inputs and Outputs driver module.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#include "drvGPIO.h"

#include <stdio.h>
#include <string.h>

#include <stm32f10x.h>

#include <cfgGlobal.h>

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types
/** GPIO SoC configuration structure. */
typedef struct GPIO_struct
{
    /** Bit mask to enable the clock to the GPIO. */
    unsigned int GPIO_clockEnableBitMask;

    /** Pointer to the CMSIS GPIO structure. */
    GPIO_TypeDef* GPIOx;
} GPIO_t;

/** Instance descriptor type. */
typedef struct inst_struct
{
    BOOL started; /**< \c TRUE if instance started, \c FALSE otherwise. */
} inst_t;

//------------------------------------------------------------------------------
// Constants and Variables
/**
 * GPIO SoC configuration.
 */
static const GPIO_t GPIO[] =
{
    // instId = A
    {
        RCC_APB2ENR_IOPAEN,     // GPIO_clockEnableBitMask
        GPIOA,                  // GPIOx
    },
    // instId = B
    {
        RCC_APB2ENR_IOPBEN,     // GPIO_clockEnableBitMask
        GPIOB,                  // GPIOx
    },
    // instId = C
    {
        RCC_APB2ENR_IOPCEN,     // GPIO_clockEnableBitMask
        GPIOC,                  // GPIOx
    },
    // instId = D
    {
        RCC_APB2ENR_IOPDEN,     // GPIO_clockEnableBitMask
        GPIOD,                  // GPIOx
    },
    // instId = E
    {
        RCC_APB2ENR_IOPEEN,     // GPIO_clockEnableBitMask
        GPIOE,                  // GPIOx
    },
};

static inst_t sInstDscr[DRV_GPIO_INST_CNT]; /**< Instance descriptor. */

//------------------------------------------------------------------------------
// Local Functions
/**
 * Initializes the variables of the module instance.
 * @param[in] instId is the instance identifier.
 */
static void initInst(int instId)
{
    memset(&sInstDscr[instId], 0, sizeof(sInstDscr[instId]));
}

//------------------------------------------------------------------------------
// Global Functions
int drvGPIO_start(int instId)
{
    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);
    CHECK_NOT_STARTED_INT(sInstDscr, instId);

    initInst(instId);

    RCC->APB2ENR |= GPIO[instId].GPIO_clockEnableBitMask;

    sInstDscr[instId].started = TRUE;
    return R_SUCCESS;
}

int drvGPIO_stop(int instId)
{
    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);

    if (sInstDscr[instId].started)
    {
        /* Don't disable clock to GPIO port. Could be in use otherwise. */

        sInstDscr[instId].started = FALSE;
    }
    return R_SUCCESS;
}

int drvGPIO_clrOutputBits(int instId, unsigned short bitPattern)
{
    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);

    GPIO[instId].GPIOx->BSRR |= (unsigned int)bitPattern << 16;
    return R_SUCCESS;
}

int drvGPIO_configPin(int instId, const drvGPIO_pinAttr_t* pinAttr)
{
    short bitNumber;
    volatile unsigned int* GPIO_CR; // GPIO CRH or CRL

    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(pinAttr);
    CHECK_RANGE_INT((int)pinAttr->IO_mode, 0, DRV_GPIO_IO_MODE_CNT);
    CHECK_RANGE_INT(pinAttr->outMaxFreq, 1, DRV_GPIO_OUT_MAX_FREQ_CNT);

    /* Determine the Port Configuration register. */
    for (bitNumber = 0; bitNumber < 16; bitNumber++)
    {
        if ((1UL << bitNumber) & pinAttr->bitPattern)
        {
            if (bitNumber < 8)
            {
                GPIO_CR = &GPIO[instId].GPIOx->CRL;
            }
            else
            {
                GPIO_CR = &GPIO[instId].GPIOx->CRH;
            }

            /* Reset the relevant bits in the Port Configuration register. */
            *GPIO_CR &= ~(0xFUL << (bitNumber % 8) * 4);

            /* Set the relevant bits in the Port Configuration register. */
            switch (pinAttr->IO_mode)
            {
            case DRV_GPIO_IO_MODE_AF_OUT_OPEN_DRAIN:
                *GPIO_CR |= (0xCUL + pinAttr->outMaxFreq) << (bitNumber % 8) * 4;
                break;

            case DRV_GPIO_IO_MODE_AF_OUT_PUSH_PULL:
                *GPIO_CR |= (0x8UL + pinAttr->outMaxFreq) << (bitNumber % 8) * 4;
                break;

            case DRV_GPIO_IO_MODE_IN_ANALOG:
                // All 4 bits cleared configures the pin as analog input
                break;

            case DRV_GPIO_IO_MODE_IN_FLOATING:
                *GPIO_CR |= 0x4UL << (bitNumber % 8) * 4;
                break;

            case DRV_GPIO_IO_MODE_IN_PULL_DOWN:
                *GPIO_CR |= 0x8UL << (bitNumber % 8) * 4;
                GPIO[instId].GPIOx->ODR &= ~(1UL << bitNumber);
                break;

            case DRV_GPIO_IO_MODE_IN_PULL_UP:
                *GPIO_CR |= 0x8UL << (bitNumber % 8) * 4;
                GPIO[instId].GPIOx->ODR |= 1UL << bitNumber;
                break;

            case DRV_GPIO_IO_MODE_OUT_OPEN_DRAIN:
                *GPIO_CR |= (0x4UL + pinAttr->outMaxFreq) << (bitNumber % 8) * 4;
                break;

            case DRV_GPIO_IO_MODE_OUT_PUSH_PULL:
                *GPIO_CR |= (0x0UL + pinAttr->outMaxFreq) << (bitNumber % 8) * 4;
                break;

            default:
                PRINT_ERROR("Unsupported I/O mode: %i", pinAttr->IO_mode);
                return R_ERROR;
            }
        }
    }
    return R_SUCCESS;
}

int drvGPIO_readIO_state(int instId, unsigned short* value)
{
    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(value);

    *value = (unsigned short)GPIO[instId].GPIOx->IDR;
    return R_SUCCESS;
}

int drvGPIO_setOutputBits(int instId, unsigned short bitPattern)
{
    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);

    GPIO[instId].GPIOx->BSRR |= (unsigned int)bitPattern;
    return R_SUCCESS;
}
