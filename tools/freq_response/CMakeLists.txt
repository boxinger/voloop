add_executable(voloop_freq_response_runner
    runner/main.c
    runner/vfr_fake_subjects.c
    runner/modules/vfr_fof_adapter.c
    runner/modules/vfr_pid_adapter.c
    runner/modules/vfr_qpr_adapter.c
    runner/vfr_point_measure.c
)

target_include_directories(voloop_freq_response_runner
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/runner
        ${CMAKE_CURRENT_SOURCE_DIR}/runner/modules
)

target_compile_features(voloop_freq_response_runner
    PRIVATE
        c_std_99
)

set_target_properties(voloop_freq_response_runner PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

target_link_libraries(voloop_freq_response_runner
    PRIVATE
        voloop::voloop
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(voloop_freq_response_runner PRIVATE m)
endif()

if(VOLOOP_BUILD_TESTS)
    add_executable(vfr_point_measure_test
        runner/test_point_measure_main.c
        runner/vfr_point_measure.c
    )

    target_include_directories(vfr_point_measure_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/runner
    )

    target_compile_features(vfr_point_measure_test
        PRIVATE
            c_std_99
    )

    set_target_properties(vfr_point_measure_test PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF
    )

    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_link_libraries(vfr_point_measure_test PRIVATE m)
    endif()

    add_test(
        NAME vfr_point_measure_test
        COMMAND $<TARGET_FILE:vfr_point_measure_test>
    )

    set_tests_properties(vfr_point_measure_test PROPERTIES
        LABELS "L2;property;tool;freq_response"
    )
endif()
