/**
 *  Filename:       main.c
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

#include <stdio.h>

#include <stm32f4xx.h>

#include <cfgGlobal.h>
#include <cfgPlatform.h>
#include "cfgProject.h"

#include "drvGPIO.h"
#include "drvSysTick.h"
#include "init.h"
#include "prpLine.h"

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

/** Application title string. */
static const char strApplicationTitle[] =
    "\n\n"
    "STM32F401 UART Example V1.0\n"
    "===========================\n";

/** Application trailer string. */
static const char strApplicationTrailer[] =
    "Application terminated.\n";

//------------------------------------------------------------------------------
// Local Functions

//------------------------------------------------------------------------------
// Global Functions
/**
 * STM32F401 UART Example Main Function.
 * @return Final error code.
 */
int main(void)
{
    char line[128]; // Within 10ms not more than 128 chars can be received
    BOOL run;
    drvUART_serialAttr_t serialAttr;
    int status;
    int tickCnt;

    /* Initialize the most basic items in the controller. */
    initBasics();
    initConsoleUSART();

    printf(strApplicationTitle);

    /* Start drivers. */
    printf("Starting driver modules... ");
    if (drvSysTick_start(1000) != R_SUCCESS)
    {
        return R_ERROR;
    }

    if (drvGPIO_start(cfgDrvUART[CFG_DRVUART_INST_N1].instIdGPIO) != R_SUCCESS)
    {
        return R_ERROR;
    }
    serialAttr.baudrate = 115200;
    serialAttr.parity = 0;
    serialAttr.stopBits = 1;
    if (drvUART_start(CFG_DRVUART_INST_N1, &serialAttr) != R_SUCCESS)
    {
        return R_ERROR;
    }
    printf("done.\n");

    /* Start Data Preparation modules. */
    printf("Starting data preparation modules... ");
    prpLine_start();
    printf("done.\n");

    /* Output some instructions to the debug console. */
    printf("Entering the main loop.\n\n");
    printf("Configure a terminal program with 115200/8/N/1.\n");
    printf("In addition configure it to issue a '\\n' character when\n");
    printf("the <Return> key is pressed.\n");
    printf("Connect it to the drvUART instance %i.\n", CFG_DRVUART_INST_N1);
    printf("Enter some text on the terminal program.\n");
    printf("Every time <Return> is pressed, the board will return a line.\n\n");

    /* The MAIN loop. */
    tickCnt = 0;
    run = TRUE;
    while (run)
    {
        drvSysTick_waitForTick();
        tickCnt += 1;
        
        if (tickCnt >= CFG_CYCLE_PERIOD_LONG / CFG_CYCLE_PERIOD_SHORT)
        {
            tickCnt = 0;
            /* Execute less frequent handler functions. */
            prpLine_handler();
            
            /* As an example, read out the data received and prepared by
             * the prpLine module and write it back to the UART driver.
             */
            if ((status = prpLine_readLine(line, sizeof(line))) > 0)
            {
                drvUART_write(CFG_DRVUART_INST_N1, line, status);
            }
        }
    }
    
    /* Stop the module. */
    prpLine_stop();
    drvUART_stop(CFG_DRVUART_INST_N1);
    drvGPIO_stop(cfgDrvUART[CFG_DRVUART_INST_N1].instIdGPIO);
    drvSysTick_stop();

    printf(strApplicationTrailer);
    return 0;
}
