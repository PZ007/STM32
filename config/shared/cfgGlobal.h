/**
 *  Filename:       cfgGlobal.h
 *  Platform(s):    All
 *  Project:
 *  Created:        Nov 12, 2012
 *  Description:    Global configuration module.
 *  Notes:
 *  Author:         Andreas Isenegger
 *  Copyright:      2013-2014, Bitcontrol GmbH, Switzerland.
 *                  Copyright granted under the MIT License.
 *                  See http://opensource.org/licenses/MIT for the license text.
 */

#ifndef CFGGLOBAL_H_
#define CFGGLOBAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <cfgPlatform.h>

//------------------------------------------------------------------------------
// Symbols and Macros
#ifndef BOOL
#define BOOL                            int
#endif
#ifndef FALSE
#define FALSE                           0
#endif
#ifndef TRUE
#define TRUE                            1
#endif

/** Return value of a function in case of an error. */
#define R_ERROR                         -1

/** Return value of a function in case of success (no error). */
#define R_SUCCESS                       1

/** Checks if the instance of the module hasn't been started. */
#define CHECK_NOT_STARTED_INT(array, instId) \
    if ((array)[(instId)].started) { \
        PRINT_ERROR("Instance already started (instId=%i)", \
            (instId)); \
        return R_ERROR; \
    }

/** Checks if the pointer isn't NULL. */
#define CHECK_POINTER_INT(ptr) \
    if ((ptr) == (void*)0) { \
        PRINT_ERROR("NULL pointer check failed"); \
        return R_ERROR; \
    }

/** Checks if the pointer isn't NULL. */
#define CHECK_POINTER_VOID(ptr) \
    if ((ptr) == (void*)0) { \
        PRINT_ERROR("NULL pointer check failed"); \
        return; \
    }

/** Checks if the variable is inside the specified range. */
#define CHECK_RANGE_INT(var, min, max) \
    if ((var)<(min) || (var)>=(max)) { \
        PRINT_ERROR("Range check failed (var=%i, min=%i, max=%i)", \
            (var), (min), (max)-1); \
        return R_ERROR; \
    }

/** Checks if the variable is inside the specified range. */
#define CHECK_RANGE_VOID(var, min, max) \
    if ((var)<(min) || (var)>=(max)) { \
        PRINT_ERROR("Range check failed (var=%i, min=%i, max=%i)", \
            (var), (min), (max)-1); \
        return; \
    }

/** Checks if the instance of the module has been started. */
#define CHECK_STARTED_INT(array, instId) \
    if (!(array)[(instId)].started) { \
        PRINT_ERROR("Start check failed (instId=%i)", \
            (instId)); \
        return R_ERROR; \
    }

/** Checks if the instance of the module has been started. */
#define CHECK_STARTED_VOID(array, instId) \
    if (!(array)[(instId)].started) { \
        PRINT_ERROR("Start check failed (instId=%i)", \
            (instId)); \
        return; \
    }

//------------------------------------------------------------------------------
// Types

//------------------------------------------------------------------------------
// Constants and Variables

//------------------------------------------------------------------------------
// Functions

#ifdef __cplusplus
}
#endif

#endif /* CFGGLOBAL_H_ */
