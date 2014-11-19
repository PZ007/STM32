/**
 *  Filename:       drvUART.c
 *  Platform(s):    STM32F10x
 *  Project:
 *  Created:        Apr 25, 2013
 *  Description:    Universal Asynchronous Receiver Transmitter driver module
 *                  using the interrupt mechanism for reading data from and
 *                  writing data to memory.
 *
 *  Notes:          USART stands for <em> Universal Synchronous and Asynchronous
 *                  Receiver Transmitter </em>.
 *
 *  Author:         Andreas Isenegger
 *  Copyright:      2013-2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#include "drvUART.h"

#include <stdio.h>
#include <string.h>

#include <misc.h>
#include <stm32f10x.h>

#include <cfgGlobal.h>
#include <cfgPlatform.h>
#include "cfgProject.h"

#include "drvGPIO.h"

//------------------------------------------------------------------------------
// Symbols and Macros
/* Copied from stm32f4xx_hal_uart.h. */
#define DIV_SAMPLING16(_PCLK_, _BAUD_) \
    (((_PCLK_)*25)/(4*(_BAUD_)))

#define DIVMANT_SAMPLING16(_PCLK_, _BAUD_) \
    (DIV_SAMPLING16((_PCLK_), (_BAUD_))/100)

#define DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_) \
    (((DIV_SAMPLING16((_PCLK_), (_BAUD_)) - \
    (DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100)) * 16 + 50) / 100)

#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_) \
    ((DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4) \
    |(DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0F))

//------------------------------------------------------------------------------
// Types
/** Instance descriptor type. */
typedef struct inst_struct
{
    drvUART_errHandler_f errHandler; /**< Pointer to error handler function. */
    short rxBufIdx; /**< Receive buffer index. */
    BOOL started; /**< \c TRUE if instance started, \c FALSE otherwise. */
    short txBufIdx; /**< Transmit buffer index. */
    short txBufByteCnt; /**< Transmit buffer byte count. */
} inst_t;

//------------------------------------------------------------------------------
// Constants and Variables
static inst_t sInstDscr[CFG_DRVUART_INST_CNT]; /**< Instance descriptor. */

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

static void interruptHandler(int instId)
{
    unsigned char byte;
    int error;
    volatile unsigned short CR1; // Control Register 1
    volatile unsigned short SR; // Status Register

    error = 0;
    CR1 = cfgDrvUART[instId].USARTx->CR1;
    SR = cfgDrvUART[instId].USARTx->SR;
    if (CR1 & USART_CR1_RXNEIE)
    {
        if (SR & USART_SR_ORE) // Overrun error?
        {
            error |= DRV_UART_ORE;
        }
        if (SR & USART_SR_PE) // Parity error?
        {
            error |= DRV_UART_PE;
        }
        if ((SR & USART_SR_RXNE) || (SR & USART_SR_ORE))
        {
            byte = cfgDrvUART[instId].USARTx->DR;

            /* Is there still space in the receive buffer? */
            if (sInstDscr[instId].rxBufIdx < cfgDrvUART[instId].rxBufferSize)
            {
                cfgDrvUART[instId].rxBuffer[sInstDscr[instId].rxBufIdx++] =
                    byte;
            }
            else // Discard byte
            {
                error |= DRV_UART_RBF;
            }
        }
    }
    if((CR1 & USART_CR1_TXEIE) && (SR & USART_SR_TXE))
    {
        /* Is there still data in the transmit buffer? */
        if (sInstDscr[instId].txBufByteCnt > sInstDscr[instId].txBufIdx)
        {
            cfgDrvUART[instId].USARTx->DR = cfgDrvUART[instId].txBuffer[
                sInstDscr[instId].txBufIdx++];
        }
        else
        {
            cfgDrvUART[instId].USARTx->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    if (error != 0 && sInstDscr[instId].errHandler != NULL)
    {
        sInstDscr[instId].errHandler(error);
    }
}

//------------------------------------------------------------------------------
// Global Functions
void USART1_IRQHandler(void)
{
    int i;
    for (i = 0; i < CFG_DRVUART_INST_CNT; i++)
    {
        if (cfgDrvUART[i].USARTx == USART1)
        {
            interruptHandler(i);
            break;
        }
    }
}

void USART2_IRQHandler(void)
{
    int i;
    for (i = 0; i < CFG_DRVUART_INST_CNT; i++)
    {
        if (cfgDrvUART[i].USARTx == USART2)
        {
            interruptHandler(i);
            break;
        }
    }
}

void USART3_IRQHandler(void)
{
    int i;
    for (i = 0; i < CFG_DRVUART_INST_CNT; i++)
    {
        if (cfgDrvUART[i].USARTx == USART3)
        {
            interruptHandler(i);
            break;
        }
    }
}

int drvUART_start(int instId, const drvUART_serialAttr_t* attr)
{
    unsigned int APB_clkFreq;
    drvGPIO_pinAttr_t GPIO_pinAttr;
    NVIC_InitTypeDef NVIC_initStruct;
    int status;

    CHECK_RANGE_INT(instId, 0, CFG_DRVUART_INST_CNT);
    CHECK_NOT_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(attr);

    CHECK_RANGE_INT(attr->baudrate, DRV_UART_BAUDRATE_MIN,
        DRV_UART_BAUDRATE_MAX + 1);
    CHECK_RANGE_INT(attr->parity, 0, 3);
    CHECK_RANGE_INT(attr->stopBits, 1, 3);

    initInst(instId);

    /* Configure assigned NVIC IRQ channel. */
    NVIC_initStruct.NVIC_IRQChannel = cfgDrvUART[instId].NVIC_IRQ_channel;
    NVIC_initStruct.NVIC_IRQChannelPreemptionPriority =
        cfgDrvUART[instId].NVIC_IRQ_groupPriority;
    NVIC_initStruct.NVIC_IRQChannelSubPriority =
        cfgDrvUART[instId].NVIC_IRQ_subPriority;
    NVIC_initStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_initStruct);

    /* Enable clock to USART. */
    *cfgDrvUART[instId].RCC_clockEnableRegister |=
        cfgDrvUART[instId].RCC_clockEnableBitMask;

    /* Initialize USART. */

    /* Set baud rate. */
    if (cfgDrvUART[instId].RCC_clockEnableRegister == &RCC->APB1ENR)
    {
        APB_clkFreq = CFG_GET_APB1_CLK();
    }
    else
    {
        APB_clkFreq = CFG_GET_APB2_CLK();
    }
    cfgDrvUART[instId].USARTx->BRR = UART_BRR_SAMPLING16(APB_clkFreq,
        attr->baudrate);

    /* Set parity. */
    switch (attr->parity)
    {
    case 0: // No parity
        // Register bits remain 0
        break;
    case 1: // Odd parity
        cfgDrvUART[instId].USARTx->CR1 |= USART_CR1_M | USART_CR1_PCE
            | USART_CR1_PS;
        break;
    case 2: // Even parity
        cfgDrvUART[instId].USARTx->CR1 |= USART_CR1_M | USART_CR1_PCE;
        break;
    default:
        /* These cases are covered by the CHECK_RANGE_INT(attr->parity, 0, 3)
         * statement above, thus nothing to do here.
         */
        break;
    }

    /* Set stop bit count. */
    switch (attr->stopBits)
    {
    case 1: // 1 stop bit
        // Register bits remain zero
        break;
    case 2: // 2 stop bits
        cfgDrvUART[instId].USARTx->CR2 |= USART_CR2_STOP_1;
        break;
    default:
        /* 0.5 and 1.5 stop bits are also supported by the SoC, but not
         * yet by this driver.
         * These cases are covered by the CHECK_RANGE_INT(attr->stopBits, 1, 3)
         * statement above, thus nothing to do here.
         */
        break;
    }

    /* Enable USART. */
    cfgDrvUART[instId].USARTx->CR1 |= USART_CR1_RE | USART_CR1_TE
        | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TXEIE;

    /* Configure dedicated GPIO port pins.
     * Note: initially this was done before configuring the USART. Then however
     * the dedicated output pin generates an unclean low level signal for some
     * ten microseconds, which results in a ghost character on the line.
     */
    GPIO_pinAttr.bitPattern =
        1 << cfgDrvUART[instId].GPIO_portBitNumberTx;
    GPIO_pinAttr.IO_mode = DRV_GPIO_IO_MODE_AF_OUT_PUSH_PULL;
    GPIO_pinAttr.outMaxFreq = DRV_GPIO_OUT_MAX_FREQ_50MHZ;
    if ((status = drvGPIO_configPin(cfgDrvUART[instId].instIdGPIO,
        &GPIO_pinAttr)) == R_SUCCESS)
    {
        GPIO_pinAttr.bitPattern =
            1 << cfgDrvUART[instId].GPIO_portBitNumberRx;
        GPIO_pinAttr.IO_mode = DRV_GPIO_IO_MODE_IN_PULL_UP;
        if ((status = drvGPIO_configPin(cfgDrvUART[instId].instIdGPIO,
            &GPIO_pinAttr)) == R_SUCCESS)
        {
            sInstDscr[instId].started = TRUE;
            return R_SUCCESS;
        }
        else
        {
            PRINT_ERROR("[%i]: drvGPIO_configPin() returned %i", instId, status);
        }
    }
    else
    {
        PRINT_ERROR("[%i]: drvGPIO_configPin() returned %i", instId, status);
    }
    return R_ERROR;
}

int drvUART_stop(int instId)
{
    drvGPIO_pinAttr_t GPIO_pinAttr;
    NVIC_InitTypeDef NVIC_initStruct;
    int returnStatus;
    int status;

    CHECK_RANGE_INT(instId, 0, CFG_DRVUART_INST_CNT);

    returnStatus = R_SUCCESS;
    if (sInstDscr[instId].started)
    {
        /* Configure GPIO port pins as inputs. */
        GPIO_pinAttr.bitPattern =
            1 << cfgDrvUART[instId].GPIO_portBitNumberRx |
            1 << cfgDrvUART[instId].GPIO_portBitNumberTx;
        GPIO_pinAttr.IO_mode = DRV_GPIO_IO_MODE_IN_FLOATING;
        /* Is ignored for inputs, but initialize member with valid value. */
        GPIO_pinAttr.outMaxFreq = DRV_GPIO_OUT_MAX_FREQ_10MHZ;
        if ((status = drvGPIO_configPin(cfgDrvUART[instId].instIdGPIO,
            &GPIO_pinAttr)) != R_SUCCESS)
        {
            PRINT_ERROR("[%i]: drvGPIO_configPin() returned %i",
                instId, status);
            returnStatus = status;
        }

        /* Disable USART. */
        cfgDrvUART[instId].USARTx->CR1 &= ~(USART_CR1_RE | USART_CR1_TE
            | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TXEIE);

        /* Clear the stop bits in the register. */
        cfgDrvUART[instId].USARTx->CR2 &= ~USART_CR2_STOP;

        /* Clear the parity bits in the register. */
        cfgDrvUART[instId].USARTx->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);

        /* Disable clock to USART. */
        *cfgDrvUART[instId].RCC_clockEnableRegister &=
            ~cfgDrvUART[instId].RCC_clockEnableBitMask;

        /* Disable assigned NVIC IRQ channel. */
        NVIC_initStruct.NVIC_IRQChannel = cfgDrvUART[instId].NVIC_IRQ_channel;
        NVIC_initStruct.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&NVIC_initStruct);
    }
    sInstDscr[instId].started = FALSE;
    return returnStatus;
}

int drvUART_isTxIdle(int instId, BOOL* result)
{
    CHECK_RANGE_INT(instId, 0, CFG_DRVUART_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(result);

    if (cfgDrvUART[instId].USARTx->SR & USART_SR_TC)
    {
        *result = TRUE;
    }
    else
    {
        *result = FALSE;
    }
    return R_SUCCESS;
}

int drvUART_read(int instId, char* dstBuf, int bufSize)
{
    volatile unsigned short CR1; // Control Register 1
    int sizeRead; // Number of bytes read

    CHECK_RANGE_INT(instId, 0, CFG_DRVUART_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(dstBuf);

    /* Save the current receive interrupt enable state. */
    CR1 = cfgDrvUART[instId].USARTx->CR1;

    /* Disable receive interrupts. */
    cfgDrvUART[instId].USARTx->CR1 &= ~USART_CR1_RXNEIE;

    /* Read the received data if any. */
    if (bufSize >= sInstDscr[instId].rxBufIdx) // Destination large enough?
    {
        memcpy(dstBuf, cfgDrvUART[instId].rxBuffer, sInstDscr[instId].rxBufIdx);
        sizeRead = sInstDscr[instId].rxBufIdx;
        sInstDscr[instId].rxBufIdx = 0;
    }
    else // Destination too small to hold all received data
    {
        memcpy(dstBuf, cfgDrvUART[instId].rxBuffer, bufSize);
        sizeRead = bufSize;
        memmove(&cfgDrvUART[instId].rxBuffer[0],
            &cfgDrvUART[instId].rxBuffer[bufSize],
            sInstDscr[instId].rxBufIdx - bufSize);
        sInstDscr[instId].rxBufIdx = sInstDscr[instId].rxBufIdx - bufSize;
    }

    /* Restore the interrupt enable state. */
    cfgDrvUART[instId].USARTx->CR1 |= USART_CR1_RXNEIE;

    return sizeRead;
}

int drvUART_registerErrorHandler(int instId, drvUART_errHandler_f errHandler)
{
    CHECK_RANGE_INT(instId, 0, CFG_DRVUART_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);

    sInstDscr[instId].errHandler = errHandler;
    return R_SUCCESS;
}

int drvUART_write(int instId, const char *srcBuf, int dataSize)
{
    int remainingBytes; // Remaining bytes to be transmitted
    int sizeWritten; // Number of bytes written

    CHECK_RANGE_INT(instId, 0, CFG_DRVUART_INST_CNT);
    CHECK_STARTED_INT(sInstDscr, instId);
    CHECK_POINTER_INT(srcBuf);

    /* Disable transmit interrupts. */
    cfgDrvUART[instId].USARTx->CR1 &= ~USART_CR1_TXEIE;

    /* Calculate the remaining bytes to be transmitted. */
    remainingBytes = sInstDscr[instId].txBufByteCnt
        - sInstDscr[instId].txBufIdx;

    /* Calculate free space in transmit buffer. */
    if (cfgDrvUART[instId].txBufferSize - remainingBytes >= dataSize)
        // Tx buffer large enough?
    {
        sizeWritten = dataSize;
    }
    else // Tx buffer too small to hold all data to be transmitted
    {
        sizeWritten = cfgDrvUART[instId].txBufferSize - remainingBytes;
    }

    /* Move remaining data in transmit buffer to beginning of buffer. */
    memmove(cfgDrvUART[instId].txBuffer,
        &cfgDrvUART[instId].txBuffer[sInstDscr[instId].txBufIdx],
        remainingBytes);

    /* Copy new data into transmit buffer. */
    memcpy(&cfgDrvUART[instId].txBuffer[remainingBytes],
        srcBuf, sizeWritten);

    /* Update buffer management variables. */
    sInstDscr[instId].txBufByteCnt = remainingBytes + sizeWritten;
    sInstDscr[instId].txBufIdx = 0;

    /* Enable transmit interrupts. */
    cfgDrvUART[instId].USARTx->CR1 |= USART_CR1_TXEIE;

    return sizeWritten;
}
