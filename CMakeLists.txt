cmake_minimum_required(VERSION 3.22)

project(voloop LANGUAGES C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(voloop_options)

add_subdirectory(core)
add_subdirectory(extensions)

if(VOLOOP_BUILD_TESTS)
    include(CTest)
endif()

if(VOLOOP_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(VOLOOP_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(VOLOOP_BUILD_DOCS)
    add_subdirectory(docs)
endif()
