cmake_minimum_required(VERSION 3.22)

project(voloop_freq_response_tools LANGUAGES C)

add_executable(vfr_point_measure_test
    runner/test_point_measure_main.c
    runner/vfr_point_measure.c
)

add_executable(voloop_freq_response_runner
    runner/main.c
    runner/vfr_fake_subjects.c
    runner/vfr_point_measure.c
)

target_include_directories(vfr_point_measure_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/runner
)

target_include_directories(voloop_freq_response_runner
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/runner
)

target_compile_features(vfr_point_measure_test
    PRIVATE
        c_std_99
)

target_compile_features(voloop_freq_response_runner
    PRIVATE
        c_std_99
)

set_target_properties(vfr_point_measure_test PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

set_target_properties(voloop_freq_response_runner PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(vfr_point_measure_test PRIVATE m)
    target_link_libraries(voloop_freq_response_runner PRIVATE m)
endif()
