cmake_minimum_required(VERSION 3.22)

project(voloop LANGUAGES C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(voloop_options)

add_subdirectory(core)

if(VOLOOP_BUILD_TESTS)
    include(CTest)
    add_subdirectory(tests)
endif()
