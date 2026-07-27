# voloop test architecture:
#
# L0 smoke:
#   Compile, link, and public-header integration tests.
#   Failure means the library is not usable and must block CI.
#
# L1 guard:
#   Lifecycle, state-guard, error-state, and protection-state tests.
#   Failure means unsafe API or state-machine behavior and must be fixed.
#
# L2 property:
#   Module-level behavior and contract tests without plant models.
#   These tests verify invariants, saturation behavior, anti-windup behavior,
#   equivalent initialization behavior, simple input/output properties, and regressions.
#   Failure means a module violates its expected behavior.
#
# L3 sim:
#   Model-based or external-script closed-loop simulation tests.
#   These tests verify system-level control behavior with plant models, disturbances,
#   convergence, lock, steady-state error, overshoot, and stability.
#   Failure means closed-loop behavior needs analysis and may be gated separately.

include(voloop_testing)

voloop_add_c_test(voloop_link_smoke
    SOURCES
        smoke/link_smoke.c
    LABELS
        L0
        smoke
)

voloop_add_c_test(voloop_pid_guard
    SOURCES
        pid/test_pid_guard.c
    LABELS
        L1
        guard
        pid
)

voloop_add_c_test(voloop_pid_property
    SOURCES
        pid/test_pid_property.c
    LABELS
        L2
        property
        pid
)

voloop_add_c_test(voloop_3phoffinv_guard
    SOURCES
        3phoffinv/test_3phoffinv_guard.c
    LABELS
        L1
        guard
        3phoffinv
)

voloop_add_c_test(voloop_3phoffinv_property
    SOURCES
        3phoffinv/test_3phoffinv_property.c
    LABELS
        L2
        property
        3phoffinv
)
