cmake_minimum_required(VERSION 3.22)

add_library(voloop STATIC)

target_sources(voloop PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_buck.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_def.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_nco.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_pll.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Src/voloop_fof.c
)

target_include_directories(voloop PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Inc
)