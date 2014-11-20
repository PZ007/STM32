/**
 *  Filename:       drvSysTick.h
 *  Platform(s):    STM32F10x
 *  Project:
 *  Created:        Mar 25, 2013
 *  Description:    System Tick driver module.
 *
 *                  This module provides a periodic timer that can be used as
 *                  system tick source. It uses the STM32F10x internal System
 *                  Tick Timer as timer base.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2013-2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#ifndef DRVSYSTICK_H_
#define DRVSYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <cfgPlatform.h>

//------------------------------------------------------------------------------
// Symbols and Macros
/** Longest period of the SysTick Timer in microseconds. */
#define DRV_SYSTICK_PERIOD_US_MAX       500000

/** Shortest period of the SysTick Timer in microseconds. */
#define DRV_SYSTICK_PERIOD_US_MIN       100

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Functions
/**
 * Starts the driver instance.
 * @param[in] period_us is the period of the timer in microseconds.
 * @return \c R_SUCCESS if the timer could be started, \c R_ERROR otherwise.
 */
int drvSysTick_start(unsigned int period_us);

/**
 * Stops the driver instance.
 * @return \c R_SUCCESS in any case.
 */
int drvSysTick_stop(void);

/**
 * Returns the number of missed system ticks since the start of the timer.
 * A tick is being considered as missed if drvSysTick_waitForTick() hasn't
 * been called between two consecutive ticks.
 * @param[out] missed points to the variable that will hold the number of
 * missed system ticks since the start of the timer. In case of an error the
 * variable won't be changed.
 * @return \c R_SUCCESS if the parameter checks pass, \c R_ERROR otherwise.
 */
int drvSysTick_getMissed(unsigned long long* missed);

/**
 * Returns the period of the System Tick Timer.
 * @param[out] period_us points to the variable that will hold the period of
 * the System Tick Timer in microseconds. In case of an error the variable
 * won't be changed.
 * @return \c R_SUCCESS if the parameter checks pass, \c R_ERROR otherwise.
 */
int drvSysTick_getPeriod(unsigned int* period_us);

/**
 * Returns the uptime of the driver instance.
 * If the driver instance has been started at the beginning of the application,
 * then this function returns the uptime of the application.
 * @param[out] uptime points to the variable that will hold the uptime of the
 * driver instance. In case of an error the variable won't be changed.
 * @return \c R_SUCCESS if the parameter checks pass, \c R_ERROR otherwise.
 */
int drvSysTick_getUptime(struct timeval* uptime);

/**
 * Waits for tick of the System Tick Timer.
 * This function blocks until the System Tick Timer expires.
 * @return \c R_SUCCESS if the timer is running, \c R_ERROR otherwise.
 */
int drvSysTick_waitForTick(void);

#ifdef __cplusplus
}
#endif

#endif /* DRVSYSTICK_H_ */
