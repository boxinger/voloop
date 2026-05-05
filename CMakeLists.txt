cmake_minimum_required(VERSION 3.16)
project(voloop LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(voloop STATIC
    source/voloop_pid.c
    source/voloop_buck.c
    source/voloop_scale.c
    source/voloop_signal.c
    source/voloop_wrap.c
)

target_include_directories(voloop PUBLIC include)
