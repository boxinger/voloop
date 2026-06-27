cmake_minimum_required(VERSION 3.22)

project(voloop LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(voloop STATIC)

target_sources(voloop PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_buck.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_def.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_nco.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_pll.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_fof.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_qpr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_offinv.c
)

target_include_directories(voloop PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Inc
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(voloop PRIVATE m)
endif()

option(VOLOOP_BUILD_TESTS "Build voloop tests" OFF)

if(VOLOOP_BUILD_TESTS)
    enable_testing()

    add_executable(voloop_link_smoke
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/link_smoke/main.c
    )

    target_link_libraries(voloop_link_smoke PRIVATE voloop)

    add_test(NAME voloop_link_smoke COMMAND voloop_link_smoke)
endif()
