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

#include "voloop_def.h"
#include "voloop_pid.h"
#include "voloop_buck.h"
#include "voloop_pll.h"
#include "voloop_nco.h"
#include "voloop_offinv.h"
#include "voloop_fof.h"
#include "voloop_qpr.h"

#endif /* VOLOOP_H */
