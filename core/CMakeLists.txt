add_library(voloop STATIC)

add_library(voloop::voloop ALIAS voloop)

target_sources(voloop
    PRIVATE
        Src/voloop_buck.c
        Src/voloop_def.c
        Src/voloop_fof.c
        Src/voloop_nco.c
        Src/voloop_offinv.c
        Src/voloop_pfc.c
        Src/voloop_pid.c
        Src/voloop_pll.c
        Src/voloop_qpr.c
        Src/voloop_3phOffInv.c
)

target_include_directories(voloop
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_compile_features(voloop
    PUBLIC
        c_std_99
)

set_target_properties(voloop PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(voloop PRIVATE m)
endif()
