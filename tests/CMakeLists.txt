include(voloop_testing)

voloop_add_c_test(voloop_link_smoke
    link_smoke/main.c
)

set_tests_properties(voloop_link_smoke PROPERTIES
    LABELS "smoke"
)
