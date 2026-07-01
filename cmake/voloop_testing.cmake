function(voloop_add_c_test name)
    cmake_parse_arguments(ARG
        ""
        ""
        "SOURCES;LABELS"
        ${ARGN}
    )

    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "voloop_add_c_test(${name}) requires SOURCES")
    endif()

    add_executable(${name}
        ${ARG_SOURCES}
    )

    target_link_libraries(${name}
        PRIVATE
            voloop::voloop
    )

    target_include_directories(${name}
        PRIVATE
            ${PROJECT_SOURCE_DIR}/tests/support
    )

    target_compile_features(${name}
        PRIVATE
            c_std_99
    )

    add_test(
        NAME ${name}
        COMMAND $<TARGET_FILE:${name}>
    )

    if(ARG_LABELS)
        set_tests_properties(${name} PROPERTIES
            LABELS "${ARG_LABELS}"
        )
    endif()
endfunction()
