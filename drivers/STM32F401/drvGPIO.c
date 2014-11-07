/**
 *  Filename:       drvGPIO.c
 *  Platform(s):    STM32F401
 *  Project:
 *  Created:        Mar 19, 2014
 *  Description:    General Purpose Inputs and Outputs driver module.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#include "drvGPIO.h"

#include <stdio.h>
#include <string.h>

#include <stm32f4xx.h>

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
        RCC_AHB1ENR_GPIOAEN,    // GPIO_clockEnableBitMask
        GPIOA,                  // GPIOx
    },
    // instId = B
    {
        RCC_AHB1ENR_GPIOBEN,    // GPIO_clockEnableBitMask
        GPIOB,                  // GPIOx
    },
    // instId = C
    {
        RCC_AHB1ENR_GPIOCEN,    // GPIO_clockEnableBitMask
        GPIOC,                  // GPIOx
    },
    // instId = D
    {
        RCC_AHB1ENR_GPIODEN,    // GPIO_clockEnableBitMask
        GPIOD,                  // GPIOx
    },
    // instId = E
    {
        RCC_AHB1ENR_GPIOEEN,    // GPIO_clockEnableBitMask
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

    RCC->AHB1ENR |= GPIO[instId].GPIO_clockEnableBitMask;
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

    GPIO[instId].GPIOx->BSRRH |= bitPattern;
    return R_SUCCESS;
}

int drvGPIO_configPin(int instId, const drvGPIO_pinAttr_t* pinAttr)
{
    short bitNumber;

    CHECK_RANGE_INT(instId, 0, DRV_GPIO_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(pinAttr);
    CHECK_RANGE_INT((int)pinAttr->IO_mode, 0, DRV_GPIO_IO_MODE_CNT);

    for (bitNumber = 0; bitNumber < 16; bitNumber++)
    {
        if ((1 << bitNumber) & pinAttr->bitPattern)
        {
            /* Reset the relevant bits in the various registers. */
            /* I/O type: input. */
            GPIO[instId].GPIOx->MODER &= ~(0x03 << bitNumber * 2);
            /* Output type: push-pull. */
            GPIO[instId].GPIOx->OTYPER &= ~(0x01 << bitNumber);
            /* Output speed: low. */
            GPIO[instId].GPIOx->OSPEEDR &= ~(0x03 << bitNumber * 2);
            /* Pull-up/down resistors: off. */
            GPIO[instId].GPIOx->PUPDR &= ~(0x03 << bitNumber * 2);
            /* Alternate function: system. */
            GPIO[instId].GPIOx->AFR[bitNumber / 8] &=
                ~(0xFUL << ((bitNumber & 0x07) * 4));

            /* Set the relevant bits in the various registers. */
            switch (pinAttr->IO_mode)
            {
            case DRV_GPIO_IO_MODE_AF_IN_FLOATING:
                CHECK_RANGE_INT(pinAttr->alternate, 0, 16);
                GPIO[instId].GPIOx->MODER |= 0x02 << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x00 << bitNumber * 2;
                GPIO[instId].GPIOx->AFR[bitNumber / 8] |=
                    (unsigned int)(pinAttr->alternate) <<
                        ((bitNumber & 0x07) * 4) ;
                break;

            case DRV_GPIO_IO_MODE_AF_IN_PULL_DOWN:
                CHECK_RANGE_INT(pinAttr->alternate, 0, 16);
                GPIO[instId].GPIOx->MODER |= 0x02 << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x02 << bitNumber * 2;
                GPIO[instId].GPIOx->AFR[bitNumber / 8] |=
                    (unsigned int)(pinAttr->alternate) <<
                        ((bitNumber & 0x07) * 4) ;
                break;

            case DRV_GPIO_IO_MODE_AF_IN_PULL_UP:
                CHECK_RANGE_INT(pinAttr->alternate, 0, 16);
                /* Set AFR before MODER to avoid spikes on output signals. */
                GPIO[instId].GPIOx->AFR[bitNumber / 8] |=
                    (unsigned int)(pinAttr->alternate) <<
                        ((bitNumber & 0x07) * 4) ;
                GPIO[instId].GPIOx->MODER |= 0x02 << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x01 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_AF_OUT_OPEN_DRAIN:
                CHECK_RANGE_INT(pinAttr->alternate, 0, 16);
                /* Set AFR before MODER to avoid spikes on output signals. */
                GPIO[instId].GPIOx->AFR[bitNumber / 8] |=
                    (unsigned int)(pinAttr->alternate) <<
                        ((bitNumber & 0x07) * 4) ;
                GPIO[instId].GPIOx->MODER |= 0x02 << bitNumber * 2;
                GPIO[instId].GPIOx->OTYPER |= 0x01 << bitNumber;
                GPIO[instId].GPIOx->OSPEEDR |= pinAttr->outSpeed <<
                    bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x01 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_AF_OUT_PUSH_PULL:
                CHECK_RANGE_INT(pinAttr->alternate, 0, 16);
                CHECK_RANGE_INT((int)pinAttr->outSpeed, 0,
                    DRV_GPIO_OUT_SPEED_CNT);
                /* Set AFR before MODER to avoid spikes on output signals. */
                GPIO[instId].GPIOx->AFR[bitNumber / 8] |=
                    (unsigned int)(pinAttr->alternate) <<
                        ((bitNumber & 0x07) * 4) ;
                GPIO[instId].GPIOx->MODER |= 0x02 << bitNumber * 2;
                GPIO[instId].GPIOx->OTYPER |= 0x00 << bitNumber;
                GPIO[instId].GPIOx->OSPEEDR |= pinAttr->outSpeed <<
                    bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x00 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_IN_ANALOG:
                GPIO[instId].GPIOx->MODER |= 0x03 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_IN_FLOATING:
                GPIO[instId].GPIOx->MODER |= 0x00 << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x00 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_IN_PULL_DOWN:
                GPIO[instId].GPIOx->MODER |= 0x00 << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x02 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_IN_PULL_UP:
                GPIO[instId].GPIOx->MODER |= 0x00 << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x01 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_OUT_OPEN_DRAIN:
                CHECK_RANGE_INT((int)pinAttr->outSpeed, 0,
                    DRV_GPIO_OUT_SPEED_CNT);
                GPIO[instId].GPIOx->MODER |= 0x01 << bitNumber * 2;
                GPIO[instId].GPIOx->OTYPER |= 0x01 << bitNumber;
                GPIO[instId].GPIOx->OSPEEDR |= pinAttr->outSpeed 
                    << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x01 << bitNumber * 2;
                break;

            case DRV_GPIO_IO_MODE_OUT_PUSH_PULL:
                CHECK_RANGE_INT((int)pinAttr->outSpeed, 0,
                    DRV_GPIO_OUT_SPEED_CNT);
                GPIO[instId].GPIOx->MODER |= 0x01 << bitNumber * 2;
                GPIO[instId].GPIOx->OTYPER |= 0x00 << bitNumber;
                GPIO[instId].GPIOx->OSPEEDR |= pinAttr->outSpeed
                    << bitNumber * 2;
                GPIO[instId].GPIOx->PUPDR  |= 0x00 << bitNumber * 2;
                break;

            default:
                PRINT_ERROR("Unsupported I/O mode: %i", pinAttr->IO_mode);
                return R_ERROR;
            }

            switch (pinAttr->IO_mode)
            {
            case DRV_GPIO_IO_MODE_IN_ANALOG:
            case DRV_GPIO_IO_MODE_IN_FLOATING:
            case DRV_GPIO_IO_MODE_IN_PULL_DOWN:
            case DRV_GPIO_IO_MODE_IN_PULL_UP:
            case DRV_GPIO_IO_MODE_OUT_OPEN_DRAIN:
            case DRV_GPIO_IO_MODE_OUT_PUSH_PULL:
                /* Clear the 4 relevant bits in the Alternate Function
                 * register.
                 */
                GPIO[instId].GPIOx->AFR[bitNumber / 8] &=
                    ~((unsigned int)0x0F << (bitNumber & 0x07) * 4);
                break;
            default: // No error, one of the other I/O modes
                break;
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

    GPIO[instId].GPIOx->BSRRL |= bitPattern;
    return R_SUCCESS;
}
