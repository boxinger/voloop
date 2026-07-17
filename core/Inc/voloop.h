#ifndef VOLOOP_H
#define VOLOOP_H

/**
 * @file voloop.h
 * @brief Aggregate public header for the voloop control library.
 *
 * Include this header when an application needs access to all public voloop
 * modules. For smaller translation units, include only the specific module
 * header that is needed, such as voloop_pid.h or voloop_qpr.h.
 */

/**
 * @defgroup VOLOOP_CORE voloop Core
 * @brief Platform-independent digital power control algorithms and utilities.
 *
 * The core module contains reusable control algorithms and helper utilities
 * that do not depend on a specific MCU, HAL, board, or peripheral driver.
 *
 * Most applications include this aggregate header:
 *
 * @code
 * #include "voloop.h"
 * @endcode
 *
 * Core APIs are organized into functional modules such as common definitions,
 * PID, first-order filters, QPR, NCO, PLL, PFC, and off-grid inverter control.
 */

#include "voloop_def.h"
#include "voloop_pid.h"
#include "voloop_buck.h"
#include "voloop_pll.h"
#include "voloop_nco.h"
#include "voloop_offinv.h"
#include "voloop_fof.h"
#include "voloop_pfc.h"
#include "voloop_qpr.h"

#endif /* VOLOOP_H */
