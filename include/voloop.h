/**
 * @file voloop.h
 * @brief voloop — Voltage Output Loop
 *
 * Top-level include for the voloop digital power control algorithm library.
 * Including this single header exposes the full public API.
 *
 * Components:
 *  - voloop_pid        — Discrete-time PID controller with anti-windup
 *  - voloop_compensator — Type II (2P2Z) and Type III (3P3Z) compensators
 *  - voloop_filter     — Moving-average and first-order IIR low-pass filters
 */

#ifndef VOLOOP_H
#define VOLOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "voloop_pid.h"
#include "voloop_compensator.h"
#include "voloop_filter.h"

/** @defgroup voloop_version Library version */
/**@{*/
#define VOLOOP_VERSION_MAJOR 1  /**< Major version */
#define VOLOOP_VERSION_MINOR 0  /**< Minor version */
#define VOLOOP_VERSION_PATCH 0  /**< Patch version */
/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_H */
