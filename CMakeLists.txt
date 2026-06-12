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
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_offinv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_fof.c
)

target_include_directories(voloop PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Inc
)

if(UNIX)
    target_link_libraries(voloop PUBLIC m)
endif()

option(VOLOOP_BUILD_EXAMPLES "Build voloop examples" OFF)

if(VOLOOP_BUILD_EXAMPLES)
    add_executable(voloop_link_smoke
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/link_smoke/main.c
    )

    target_link_libraries(voloop_link_smoke PRIVATE voloop)
endif()