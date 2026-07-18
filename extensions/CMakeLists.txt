add_library(voloop_extensions STATIC)

add_library(voloop::extensions ALIAS voloop_extensions)

target_sources(voloop_extensions
    PRIVATE
        vfr_single_point.c
)

target_include_directories(voloop_extensions
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(voloop_extensions
    PUBLIC
        c_std_99
)

set_target_properties(voloop_extensions PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

target_link_libraries(voloop_extensions
    PUBLIC
        voloop::voloop
)
