/**
 *  Filename:       prpLine.c
 *  Platform(s):    STM32F401
 *  Project:        F401_UART_example
 *                  UART example project for STM32F401 microcontrollers
 *  Created:        Dec 19, 2014
 *  Description:    Line data preparation module.
 *                  For demonstration purposes.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2014, Bitcontrol GmbH, Switzerland.
 *                  All rights reserved.
 */

#include "prpLine.h"

#include <string.h>

#include <cfgGlobal.h>
#include <cfgPlatform.h>
#include "cfgProject.h"

#include "drvUART.h"

//------------------------------------------------------------------------------
// Symbols and Macros
/** Cycle period of this module's handler() function in milliseconds. */
#define CYCLE_PERIOD                    CFG_CYCLE_PERIOD_LONG

//------------------------------------------------------------------------------
// Types
/** Instance descriptor type. */
typedef struct inst_struct
{
    /** Receive buffer for the line currently being received. */
    char rxBuf[128];

    /**
     * Receive buffer index.
     * Points to the 1st free location in the receive buffer.
     */
    short rxBufIdx;

    /** \c TRUE if instance started, \c FALSE otherwise. */
    BOOL started;
} inst_t;

//------------------------------------------------------------------------------
// Constants and Variables
static inst_t sInstDscr; /**< Instance descriptor. */

//------------------------------------------------------------------------------
// Local Functions
/**
 * Initializes the variables of the instance.
 */
static void initInst(void)
{
    memset(&sInstDscr, 0, sizeof(sInstDscr));
}

//------------------------------------------------------------------------------
// Global Functions
int prpLine_start(void)
{
    CHECK_NOT_STARTED_INT(&sInstDscr, 0);

    initInst();

    sInstDscr.started = TRUE;
    return R_ERROR;
}

int prpLine_stop(void)
{
    if (sInstDscr.started)
    {
        sInstDscr.started = FALSE;
    }
    return R_SUCCESS;
}

void prpLine_handler(void)
{
    int freeBufferSpace;
    int status;
    char tmpBuf[8];

    CHECK_STARTED_VOID(&sInstDscr, 0);

    /* Transfer incoming data from the UART driver's receive buffer to the
     * internal receive buffer.
     */
    freeBufferSpace = sizeof(sInstDscr.rxBuf) - sInstDscr.rxBufIdx;
    if (freeBufferSpace > 0)
    {
        status = drvUART_read(CFG_DRVUART_INST_N1,
            sInstDscr.rxBuf + sInstDscr.rxBufIdx, freeBufferSpace);
        if (status > 0)
        {
            sInstDscr.rxBufIdx += status;
        }
        else if (status == R_ERROR)
        {
            PRINT_ERROR("drvUART_read() returned %i", R_ERROR);
        }
        // else no data received
    }

    /* When the internal receive buffer is full, discard potentially remaining
     * data in the UART driver's receive buffer. It would otherwise be read
     * as the remainder of an incomplete line after the next call to \c
     * _readLine().
     */
    if (sInstDscr.rxBufIdx >= sizeof(sInstDscr.rxBuf))
    {
        while (drvUART_read(CFG_DRVUART_INST_N1,
            tmpBuf, sizeof(tmpBuf)) > 0);
    }
}

int prpLine_readLine(char* dstBuf, int dstBufsize)
{
    char* pLineBreak;
    int stringLength; // Excludes the '\0' character as usual for C strings

    CHECK_STARTED_INT(&sInstDscr, 0);
    CHECK_POINTER_INT(dstBuf);

    if (dstBufsize > 0)
    {
        /* Does the line contain a line break character? */
        pLineBreak = memchr(sInstDscr.rxBuf, '\n', sInstDscr.rxBufIdx);
        if (pLineBreak != NULL)
        {
            /* Does the line fit into the destination buffer? */
            if (pLineBreak - sInstDscr.rxBuf + 2 <= dstBufsize)
            {
                memcpy(dstBuf, sInstDscr.rxBuf,
                    pLineBreak - sInstDscr.rxBuf + 1);
                dstBuf[pLineBreak - sInstDscr.rxBuf + 1] = '\0';
                stringLength = pLineBreak - sInstDscr.rxBuf + 1;
            }
            else // Line doesn't fit into destination buffer
            {
                memcpy(dstBuf, sInstDscr.rxBuf, dstBufsize - 1);
                dstBuf[dstBufsize - 1] = '\0';
                stringLength = dstBufsize - 1;
            }
            /* Remove the line from the internal buffer. */
            memmove(sInstDscr.rxBuf, pLineBreak + 1,
                sizeof(sInstDscr.rxBuf)
                    - (pLineBreak - sInstDscr.rxBuf + 1));
            sInstDscr.rxBufIdx -= pLineBreak - sInstDscr.rxBuf + 1;
        }
        else // No line break character found
        {
            if (sInstDscr.rxBufIdx >= sizeof(sInstDscr.rxBuf)) // Buffer full?
            {
                /* Does the line fit into the destination buffer? */
                if (sizeof(sInstDscr.rxBuf) + 1 <= dstBufsize)
                {
                    memcpy(dstBuf, sInstDscr.rxBuf,
                        sizeof(sInstDscr.rxBuf));
                    dstBuf[sizeof(sInstDscr.rxBuf)] = '\0';
                    stringLength = sizeof(sInstDscr.rxBuf) - 1;
                }
                else // Line doesn't fit into destination buffer
                {
                    memcpy(dstBuf, sInstDscr.rxBuf, dstBufsize - 1);
                    dstBuf[dstBufsize - 1] = '\0';
                    stringLength = dstBufsize - 1;
                }
                /* Remove the line from the internal buffer. */
                sInstDscr.rxBufIdx = 0;
            }
            else // Buffer isn't full yet, don't do anything
            {
                *dstBuf = '\0';
                stringLength = 0;
            }
        }
        return stringLength;
    }
    else
    {
        PRINT_ERROR("Arg size invalid: %i", dstBufsize);
    }
    return R_ERROR;
}
