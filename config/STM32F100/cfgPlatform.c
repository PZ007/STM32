/**
 *  Filename:       cfgPlatform.c
 *  Platform(s):    STM32F10x
 *  Project:
 *  Created:        Apr 08, 2013
 *  Description:    Platform configuration module.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2013-2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#include <cfgPlatform.h>

#include <stm32f10x.h>

//------------------------------------------------------------------------------
// Symbols and Macros

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables
/* Copied from stm32f4xx_rcc.c. Is identical to the one in stm32f4xx_hal_rcc.c.
 */
const unsigned char cfgAPB_AHB_prescalerTable[] =
    {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

//------------------------------------------------------------------------------
// Local Functions

//------------------------------------------------------------------------------
// Global Functions
